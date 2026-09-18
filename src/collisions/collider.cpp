#include <cmath>
#include <cstdint>

#include "collisions/collider.hpp"
#include "components/NewtonianState.hpp"
#include "spatial/Octree.hpp"

namespace {
    bool checkOverlap(const physics::NewtonianState& state, uint32_t bodyIndex, uint32_t otherIndex)
    {
        auto& xBodyA = state.posX[bodyIndex];
        auto& yBodyA = state.posY[bodyIndex];
        auto& zBodyA = state.posZ[bodyIndex];
        auto& rBodyA = state.radius[bodyIndex];

        auto& xBodyB = state.posX[otherIndex];
        auto& yBodyB = state.posY[otherIndex];
        auto& zBodyB = state.posZ[otherIndex];
        auto& rBodyB = state.radius[otherIndex];

        auto deltaX = xBodyA - xBodyB;
        auto deltaY = yBodyA - yBodyB;
        auto deltaZ = zBodyA - zBodyB;

        auto squaredDist = std::pow(deltaX, 2) + std::pow(deltaY, 2) + std::pow(deltaZ, 2);

        auto dist = std::sqrt(squaredDist);

        auto contactDist = rBodyA + rBodyB;
        if (dist <= contactDist) {
            return true;
        }
        return false;
    }
} // namespace

void physics::Collider::_visit(const Octree& octree, const NewtonianState& state, std::uint32_t nodeIndex,
                               std::vector<std::pair<uint32_t, uint32_t>>& out)
{
    const Node& node = octree.nodes()[nodeIndex];
    constexpr short NODE_MINIMAL_NUMBER = 2;

    if (node.firstChild != INVALID_NODE) {
        for (std::uint32_t octant = 0; octant < OCTANT_COUNT; octant += 1)
            this->_visit(octree, state, node.firstChild + octant, out);
        return;
    }

    const auto& permutation = octree.permutations();

    if (node.count < NODE_MINIMAL_NUMBER) {
        return;
    }
    for (std::uint32_t current = node.begin; current < node.begin + node.count; current += 1) {
        for (std::uint32_t b = current + 1; b < node.begin + node.count; b += 1) {
            const uint32_t i = permutation[current];
            const uint32_t j = permutation[b];
            if (checkOverlap(state, i, j))
                out.emplace_back(i, j);
        }
    }
}

std::vector<std::pair<std::uint32_t, std::uint32_t>> physics::Collider::checkCollisions(physics::NewtonianState& state,
                                                                                        Octree& octree)
{
    std::vector<std::pair<uint32_t, uint32_t>> collisions;
    collisions.reserve(octree.nodes().size() + 1);

    if (octree.nodes().size() == 0)
        return {};
    this->_visit(octree, state, FIRST_NODE, collisions);
    // std::cout << std::format("{} Collisions", collisions.size()) << std::endl;
    return collisions;
}
