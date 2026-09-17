#include "collider.hpp"
#include <cstdint>
#include "components/NewtonianState.hpp"
#include "spatial/Octree.hpp"

namespace {
    bool check_overlap(const physics::NewtonianState& state, uint32_t body_index, uint32_t other_index)
    {
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

    if (octree.nodes().size() == 0)
        return {};
    this->_visit(octree, state, FIRST_NODE, collisions);
    return collisions;
}
