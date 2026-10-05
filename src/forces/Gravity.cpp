#include "Gravity.hpp"
#include <algorithm>
#include <boost/iterator/counting_iterator.hpp>
#include <cmath>
#include <cstddef>
#include <execution>
#include <experimental/simd>
#include <vector>
#include "spatial/Octree.hpp"

//? Public methods

void physics::forces::Gravity::apply(NewtonianState& state, double /*dt*/, GravityMode mode, Octree& tree)
{
    if (state.size() == 0)
        return;

    if (mode == GravityMode::BarnesHut) {
        physics::forces::Gravity::_computeBarnesHutGravity(state, tree);
        return;
    }

    physics::forces::Gravity::_computeGravity(state);
}

physics::components::ScalarMass physics::forces::Gravity::computeScalarMass(const common::components::Mass& mass)
{
    return {physics::scalarMassOf(mass)};
}

//? Private methods

void physics::forces::Gravity::_computeGravity(NewtonianState& state)
{
    const std::size_t count = state.size();
    const std::size_t padded = state.paddedSize();

    const double* __restrict posX = state.posX.data();
    const double* __restrict posY = state.posY.data();
    const double* __restrict posZ = state.posZ.data();
    const double* __restrict mass = state.scalarMass.data();

    double* __restrict forceX = state.forceX.data();
    double* __restrict forceY = state.forceY.data();
    double* __restrict forceZ = state.forceZ.data();

    std::for_each(std::execution::par_unseq, boost::counting_iterator<std::size_t>(0),
                  boost::counting_iterator<std::size_t>(count),
                  [=](std::size_t i)
                  {
                      const simd_t myPx = posX[i];
                      const simd_t myPy = posY[i];
                      const simd_t myPz = posZ[i];
                      const simd_t myMassG = mass[i] * G;

                      simd_t accFx0 = 0.0;
                      simd_t accFy0 = 0.0;
                      simd_t accFz0 = 0.0;
                      simd_t accFx1 = 0.0;
                      simd_t accFy1 = 0.0;
                      simd_t accFz1 = 0.0;

                      const auto accumulate = [&](std::size_t j, simd_t& accFx, simd_t& accFy, simd_t& accFz)
                      {
                          const simd_t dx = simd_t(&posX[j], stdx::vector_aligned) - myPx;
                          const simd_t dy = simd_t(&posY[j], stdx::vector_aligned) - myPy;
                          const simd_t dz = simd_t(&posZ[j], stdx::vector_aligned) - myPz;
                          const simd_t massJ(&mass[j], stdx::vector_aligned);

                          const simd_t r2 = dx * dx + dy * dy + dz * dz + EPSILON2;
                          const simd_t invDist = 1.0 / stdx::sqrt(r2);
                          const simd_t mag = myMassG * massJ * invDist * invDist * invDist;

                          accFx += mag * dx;
                          accFy += mag * dy;
                          accFz += mag * dz;
                      };

                      for (std::size_t j = 0; j < padded; j += NewtonianState::BLOCK) {
                          accumulate(j, accFx0, accFy0, accFz0);
                          accumulate(j + LANES, accFx1, accFy1, accFz1);
                      }

                      forceX[i] = stdx::reduce(accFx0 + accFx1);
                      forceY[i] = stdx::reduce(accFy0 + accFy1);
                      forceZ[i] = stdx::reduce(accFz0 + accFz1);
                  });
}

void physics::forces::Gravity::_computeBarnesHutGravity(NewtonianState& state, Octree& tree)
{
    const std::vector<std::uint32_t> slots = _computeSlots(tree.permutations(), state.size());
    const MassCenters centers = _computeMassCenters(state, tree);

    std::for_each(std::execution::par, boost::counting_iterator<std::uint32_t>(0),
                  boost::counting_iterator<std::uint32_t>(static_cast<std::uint32_t>(state.size())),
                  [&](std::uint32_t body) { _computeBodyForce(state, tree, centers, slots, body); });
}

std::vector<std::uint32_t> physics::forces::Gravity::_computeSlots(const std::vector<std::uint32_t>& permutations,
                                                                   std::size_t bodyCount)
{
    std::vector<std::uint32_t> slots(bodyCount);

    for (std::uint32_t slot = 0; slot < permutations.size(); ++slot)
        slots[permutations[slot]] = slot;
    return slots;
}

auto physics::forces::Gravity::_computeMassCenters(const NewtonianState& state, const Octree& tree) -> MassCenters
{
    const auto& nodes = tree.nodes();
    const auto& permutations = tree.permutations();
    MassCenters centers(nodes.size());

    for (std::size_t index = nodes.size(); index-- > 0;) {
        const physics::Node& node = nodes[index];
        const Moment moment = _isLeaf(node) ? _leafMoment(node, state, permutations) : _internalMoment(node, centers);

        centers.set(index, moment);
    }
    return centers;
}

auto physics::forces::Gravity::_leafMoment(const physics::Node& node, const NewtonianState& state,
                                           const std::vector<std::uint32_t>& permutations) -> Moment
{
    Moment moment;
    const std::uint32_t end = node.begin + node.count;

    for (std::uint32_t slot = node.begin; slot < end; ++slot)
        moment += _bodyPoint(state, permutations[slot]);
    return moment;
}

auto physics::forces::Gravity::_internalMoment(const physics::Node& node, const MassCenters& centers) -> Moment
{
    Moment moment;

    for (std::uint32_t octant = 0; octant < physics::OCTANT_COUNT; ++octant)
        moment += centers.at(node.first_child + octant);
    return moment;
}

auto physics::forces::Gravity::_bodyPoint(const NewtonianState& state, std::uint32_t body) -> PointMass
{
    return {state.scalarMass[body], {state.posX[body], state.posY[body], state.posZ[body]}};
}

auto physics::forces::Gravity::_gravityFrom(const PointMass& source, const PointMass& target) -> Vec3
{
    const Vec3 delta = source.pos - target.pos;
    const double invDistance = 1.0 / std::sqrt(delta.norm2() + EPSILON2);
    const double mag = G * target.mass * source.mass * invDistance * invDistance * invDistance;

    return delta * mag;
}

void physics::forces::Gravity::_computeBodyForce(NewtonianState& state, const Octree& tree, const MassCenters& centers,
                                                 const std::vector<std::uint32_t>& slots, std::uint32_t body)
{
    const auto& nodes = tree.nodes();
    const auto& permutations = tree.permutations();
    const PointMass target = _bodyPoint(state, body);
    Vec3 force;
    NodeStack stack;

    stack.push(0);
    while (!stack.empty()) {
        const std::uint32_t index = stack.pop();
        const physics::Node& node = nodes[index];

        if (_isLeaf(node)) {
            force += _leafForce(node, state, permutations, body, target);
            continue;
        }

        const PointMass center = centers.at(index);
        if (!_containsSlot(node, slots[body]) && _canApproximate(node, center.pos, target.pos)) {
            force += _gravityFrom(center, target);
            continue;
        }

        _pushChildren(tree, node, stack);
    }

    state.forceX[body] = force.x;
    state.forceY[body] = force.y;
    state.forceZ[body] = force.z;
}

auto physics::forces::Gravity::_leafForce(const physics::Node& node, const NewtonianState& state,
                                          const std::vector<std::uint32_t>& permutations, std::uint32_t body,
                                          const PointMass& target) -> Vec3
{
    Vec3 force;
    const std::uint32_t end = node.begin + node.count;

    for (std::uint32_t slot = node.begin; slot < end; ++slot) {
        const std::uint32_t source = permutations[slot];
        if (source != body)
            force += _gravityFrom(_bodyPoint(state, source), target);
    }
    return force;
}

bool physics::forces::Gravity::_canApproximate(const physics::Node& node, const Vec3& center, const Vec3& target)
{
    const double distance2 = (center - target).norm2() + EPSILON2;
    const double size = node.halfSize * 2.0;

    return size * size < BARNES_HUT_THETA2 * distance2;
}

void physics::forces::Gravity::_pushChildren(const Octree& tree, const physics::Node& node, NodeStack& stack)
{
    const auto& nodes = tree.nodes();

    for (std::uint32_t octant = 0; octant < physics::OCTANT_COUNT; ++octant) {
        const std::uint32_t child = node.first_child + octant;
        if (nodes[child].count != 0)
            stack.push(child);
    }
}

bool physics::forces::Gravity::_isLeaf(const physics::Node& node)
{
    return node.first_child == physics::INVALID_NODE;
}

bool physics::forces::Gravity::_containsSlot(const physics::Node& node, std::uint32_t slot)
{
    return slot >= node.begin && slot < node.begin + node.count;
}

physics::components::InverseDistance
physics::forces::Gravity::computeInverseDistance(const physics::components::Displacement& disp)
{
    double r2 = disp.dx * disp.dx + disp.dy * disp.dy + disp.dz * disp.dz + EPSILON2;
    double invDist = 1.0 / std::sqrt(r2);

    return {invDist * invDist * invDist};
}
