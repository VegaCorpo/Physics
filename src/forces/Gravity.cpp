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
    //? Step 1: Build the octree

    const auto& permutations = tree.permutations();
    std::vector<std::uint32_t> slots(state.size());
    for (std::uint32_t slot = 0; slot < permutations.size(); ++slot)
        slots[permutations[slot]] = slot;

    //? Step 2: Compute the mass centers for each node in the octree
    const MassCenters centers = _computeMassCenters(state, tree);

    //? Step 3: Compute the gravitational force for each body using the octree and mass centers
    std::for_each(std::execution::par, boost::counting_iterator<std::uint32_t>(0),
                  boost::counting_iterator<std::uint32_t>(static_cast<std::uint32_t>(state.size())),
                  [&](std::uint32_t body) { _computeBodyForce(state, tree, centers, slots, body); });
}

physics::forces::Gravity::MassCenters physics::forces::Gravity::_computeMassCenters(const physics::NewtonianState& state, const physics::Octree& tree)
{
    const auto& nodes = tree.nodes();
    const auto& permutations = tree.permutations();
    physics::forces::Gravity::MassCenters centers {std::vector<double>(nodes.size()), std::vector<double>(nodes.size()),
                            std::vector<double>(nodes.size()), std::vector<double>(nodes.size())};

    for (std::size_t index = nodes.size(); index-- > 0;) {
        const physics::Node& node = nodes[index];
        double mass = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        if (node.first_child == physics::INVALID_NODE) {
            const std::uint32_t end = node.begin + node.count;
            for (std::uint32_t slot = node.begin; slot < end; ++slot) {
                const std::uint32_t body = permutations[slot];
                const double bodyMass = state.scalarMass[body];
                mass += bodyMass;
                x += bodyMass * state.posX[body];
                y += bodyMass * state.posY[body];
                z += bodyMass * state.posZ[body];
            }
        } else {
            for (std::uint32_t octant = 0; octant < physics::OCTANT_COUNT; ++octant) {
                const std::uint32_t child = node.first_child + octant;
                mass += centers.mass[child];
                x += centers.mass[child] * centers.x[child];
                y += centers.mass[child] * centers.y[child];
                z += centers.mass[child] * centers.z[child];
            }
        }

        centers.mass[index] = mass;
        if (mass != 0.0) {
            centers.x[index] = x / mass;
            centers.y[index] = y / mass;
            centers.z[index] = z / mass;
        }
    }
    return centers;
}

void physics::forces::Gravity::_addForce(double sourceMass, double sourceX, double sourceY, double sourceZ, double targetMass,
                double targetX, double targetY, double targetZ, double& forceX, double& forceY, double& forceZ)
{
    const double dx = sourceX - targetX;
    const double dy = sourceY - targetY;
    const double dz = sourceZ - targetZ;
    const double invDistance = 1.0 / std::sqrt(dx * dx + dy * dy + dz * dz + physics::forces::EPSILON2);
    const double mag = physics::forces::G * targetMass * sourceMass * invDistance * invDistance * invDistance;

    forceX += mag * dx;
    forceY += mag * dy;
    forceZ += mag * dz;
}

void physics::forces::Gravity::_computeBodyForce(physics::NewtonianState& state, const physics::Octree& tree,
                        const MassCenters& centers, const std::vector<std::uint32_t>& slots, std::uint32_t body)
{
    const auto& nodes = tree.nodes();
    const auto& permutations = tree.permutations();
    const double targetX = state.posX[body];
    const double targetY = state.posY[body];
    const double targetZ = state.posZ[body];
    const double targetMass = state.scalarMass[body];
    double forceX = 0.0;
    double forceY = 0.0;
    double forceZ = 0.0;
    std::vector<std::uint32_t> pending {0};

    while (!pending.empty()) {
        const std::uint32_t index = pending.back();
        pending.pop_back();
        const physics::Node& node = nodes[index];
        const bool containsTarget = slots[body] >= node.begin && slots[body] < node.begin + node.count;

        if (node.first_child == physics::INVALID_NODE) {
            const std::uint32_t end = node.begin + node.count;
            for (std::uint32_t slot = node.begin; slot < end; ++slot) {
                const std::uint32_t source = permutations[slot];
                if (source != body)
                    _addForce(state.scalarMass[source], state.posX[source], state.posY[source], state.posZ[source],
                                targetMass, targetX, targetY, targetZ, forceX, forceY, forceZ);
            }
            continue;
        }

        const double dx = centers.x[index] - targetX;
        const double dy = centers.y[index] - targetY;
        const double dz = centers.z[index] - targetZ;
        const double distance2 = dx * dx + dy * dy + dz * dz + physics::forces::EPSILON2;
        const double size = node.halfSize * 2.0;
        if (!containsTarget && size * size < BARNES_HUT_THETA2 * distance2) {
            _addForce(centers.mass[index], centers.x[index], centers.y[index], centers.z[index], targetMass,
                        targetX, targetY, targetZ, forceX, forceY, forceZ);
            continue;
        }

        for (std::uint32_t octant = 0; octant < physics::OCTANT_COUNT; ++octant) {
            const std::uint32_t child = node.first_child + octant;
            if (nodes[child].count != 0)
                pending.push_back(child);
        }
    }

    state.forceX[body] = forceX;
    state.forceY[body] = forceY;
    state.forceZ[body] = forceZ;
}

physics::components::InverseDistance
physics::forces::Gravity::computeInverseDistance(const physics::components::Displacement& disp)
{
    double r2 = disp.dx * disp.dx + disp.dy * disp.dy + disp.dz * disp.dz + EPSILON2;
    double invDist = 1.0 / std::sqrt(r2);

    return {invDist * invDist * invDist};
}
