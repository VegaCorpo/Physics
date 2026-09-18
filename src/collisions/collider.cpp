#include "collider.hpp"
#include <cmath>
#include <cstdint>
#include <iostream>
#include "components/NewtonianState.hpp"
#include "spatial/Octree.hpp"

namespace {
    bool check_overlap(const physics::NewtonianState& state, uint32_t body_index, uint32_t other_index)
    {
        auto& xBodyA = state.posX[body_index];
        auto& yBodyA = state.posY[body_index];
        auto& zBodyA = state.posZ[body_index];
        auto& rBodyA = state.radius[body_index];

        auto& xBodyB = state.posX[other_index];
        auto& yBodyB = state.posY[other_index];
        auto& zBodyB = state.posZ[other_index];
        auto& rBodyB = state.posZ[other_index];

        auto deltaX = xBodyA - xBodyB;
        auto deltaY = yBodyA - yBodyB;
        auto deltaZ = zBodyA - zBodyB;

        auto squared_dist = std::pow(deltaX, 2) + std::pow(deltaY, 2) + std::pow(deltaZ, 2);

        auto dist = std::sqrt(squared_dist);

        auto contactDist = rBodyA + rBodyB;
        if (dist <= contactDist) {
            return true;
        }
        return false;
    }
} // namespace

void physics::Collider::_visit(const Octree& octree, const NewtonianState& state, std::uint32_t node_index,
                               std::vector<std::pair<uint32_t, uint32_t>>& out)
{
    const Node& node = octree.nodes()[node_index];

    if (node.first_child != INVALID_NODE) {
        for (std::uint32_t octant = 0; octant < OCTANT_COUNT; octant += 1)
            this->_visit(octree, state, node.first_child + octant, out);
        return;
    }

    const auto& permutation = octree.permutations();

    if (node.count < 2) {
        return;
    }
    for (std::uint32_t current = node.begin; current < node.begin + node.count; current += 1) {
        for (std::uint32_t b = current + 1; b < node.begin + node.count; b += 1) {
            const uint32_t i = permutation[current];
            const uint32_t j = permutation[b];
            if (check_overlap(state, i, j))
                out.emplace_back(i, j);
        }
    }
}

std::vector<std::pair<std::uint32_t, std::uint32_t>> physics::Collider::check_collisions(physics::NewtonianState& state,
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
