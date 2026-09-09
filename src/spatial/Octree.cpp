#include "Octree.hpp"
#include <algorithm>
#include <cstdint>
#include "components/NewtonianState.hpp"
#include "utils/utils.hpp"

namespace {
    std::uint32_t getOctant(const physics::Node& node, double x, double y, double z)
    {
        return ((x >= node.centerX ? 1u : 0u) | (y >= node.centerY ? 2u : 0u) | (z >= node.centerZ ? 4u : 0u));
    }

    void computeOctantStarts(const physics::Node& parent,
                             const std::uint32_t (&bodies_per_octant)[physics::OCTANT_COUNT],
                             std::uint32_t (&octant_starts)[physics::OCTANT_COUNT])
    {
        octant_starts[0] = parent.begin;
        for (std::uint32_t octant = 1; octant < physics::OCTANT_COUNT; octant += 1) {
            octant_starts[octant] = octant_starts[octant - 1] + bodies_per_octant[octant - 1];
        }
    }
} // namespace

void physics::Octree::build(const physics::NewtonianState& state)
{
    std::vector<std::uint32_t> nodes_to_subdivide;

    this->clear();

    if (this->_initializeOctree(state) != OctreeState::OK)
        return;

    nodes_to_subdivide.push_back(0);
    while (!nodes_to_subdivide.empty()) {
        const std::uint32_t node_index = nodes_to_subdivide.back();
        nodes_to_subdivide.pop_back();

        if (this->_nodes[node_index].count <= LEAF_CAPACITY || this->_nodes[node_index].depth >= MAX_DEPTH) {
            continue;
        }
        this->_subdivide(state, node_index);

        const std::uint32_t first_child = this->_nodes[node_index].first_child;

        for (std::uint32_t octant = 0; octant < OCTANT_COUNT; octant += 1) {
            if (this->_nodes[first_child + octant].count > 0) {
                nodes_to_subdivide.push_back(first_child + octant);
            }
        }
    }
}

void physics::Octree::_subdivide(const physics::NewtonianState& state, std::uint32_t node_index)
{
    const Node parent = this->_nodes[node_index];

    std::uint32_t bodies_per_octant[OCTANT_COUNT] = {};
    std::uint32_t octant_starts[OCTANT_COUNT] = {};

    this->_countBodiesPerOctant(state, parent, bodies_per_octant);
    computeOctantStarts(parent, bodies_per_octant, octant_starts);
    this->_sortBodiesByOctant(state, parent, octant_starts);

    const std::uint32_t first_child = this->_createChildren(parent, octant_starts, bodies_per_octant);

    this->_nodes[node_index].first_child = first_child;
}

void physics::Octree::_countBodiesPerOctant(const physics::NewtonianState& state, const Node& parent,
                                            std::uint32_t (&bodies_per_octant)[OCTANT_COUNT])
{
    const std::uint32_t slots_end = parent.begin + parent.count;

    for (std::uint32_t slot = parent.begin; slot < slots_end; slot += 1) {
        const std::uint32_t body_index = this->_permutations[slot];
        const std::uint32_t octant =
            getOctant(parent, state.posX[body_index], state.posY[body_index], state.posZ[body_index]);

        bodies_per_octant[octant] += 1;
    }
}

void physics::Octree::_sortBodiesByOctant(const physics::NewtonianState& state, const Node& parent,
                                          const std::uint32_t (&octant_starts)[OCTANT_COUNT])
{
    const std::uint32_t slots_end = parent.begin + parent.count;

    std::uint32_t next_free_slot[OCTANT_COUNT] = {};

    std::copy(std::begin(octant_starts), std::end(octant_starts), std::begin(next_free_slot));
    for (std::uint32_t slot = parent.begin; slot < slots_end; slot += 1) {
        const std::uint32_t body_index = this->_permutations[slot];
        const std::uint32_t octant =
            getOctant(parent, state.posX[body_index], state.posY[body_index], state.posZ[body_index]);

        this->_buffer[next_free_slot[octant]] = body_index;
        next_free_slot[octant] += 1;
    }
    std::copy(this->_buffer.begin() + static_cast<std::ptrdiff_t>(parent.begin),
              this->_buffer.begin() + static_cast<std::ptrdiff_t>(slots_end),
              this->_permutations.begin() + static_cast<std::ptrdiff_t>(parent.begin));
}

std::uint32_t physics::Octree::_createChildren(const Node& parent, const std::uint32_t (&octant_starts)[OCTANT_COUNT],
                                               const std::uint32_t (&bodies_per_octant)[OCTANT_COUNT])
{
    auto first_child = static_cast<std::uint32_t>(this->_nodes.size());
    const double child_half_size = parent.halfSize * 0.5;

    for (std::uint32_t octant = 0; octant < OCTANT_COUNT; octant += 1) {
        Node child;

        child.centerX = parent.centerX + ((octant & 1u) ? child_half_size : -child_half_size);
        child.centerY = parent.centerY + ((octant & 2u) ? child_half_size : -child_half_size);
        child.centerZ = parent.centerZ + ((octant & 4u) ? child_half_size : -child_half_size);
        child.halfSize = child_half_size;
        child.first_child = INVALID_NODE;
        child.begin = octant_starts[octant];
        child.count = bodies_per_octant[octant];
        child.depth = parent.depth + 1;

        this->_nodes.push_back(child);
    }
    return first_child;
}

physics::OctreeState physics::Octree::_initializeOctree(const NewtonianState& state)
{
    std::uint32_t nb_bodies;

    for (std::uint32_t body_index = 0; body_index < state.size(); body_index += 1) {
        this->_permutations.push_back(body_index);
    }

    nb_bodies = this->_permutations.size();
    if (nb_bodies == 0) {
        return OctreeState::NO_BODY;
    }
    this->_buffer.resize(nb_bodies);
    this->_initializeFirstNode(state, nb_bodies);
    return OctreeState::OK;
}

void physics::Octree::_initializeFirstNode(const physics::NewtonianState& state, std::uint32_t nb_bodies)
{
    Node first_node;
    Bounds bounds;

    for (auto body_index : this->_permutations) {
        bounds.posMin.X = std::min(bounds.posMin.X, state.posX[body_index]);
        bounds.posMin.Y = std::min(bounds.posMin.Y, state.posY[body_index]);
        bounds.posMin.Z = std::min(bounds.posMin.Z, state.posZ[body_index]);

        bounds.posMax.X = std::max(bounds.posMax.X, state.posX[body_index]);
        bounds.posMax.Y = std::max(bounds.posMax.Y, state.posY[body_index]);
        bounds.posMax.Z = std::max(bounds.posMax.Z, state.posZ[body_index]);
    }

    first_node.centerX = (bounds.posMin.X + bounds.posMax.X) * 0.5;
    first_node.centerY = (bounds.posMin.Y + bounds.posMax.Y) * 0.5;
    first_node.centerZ = (bounds.posMin.Z + bounds.posMax.Z) * 0.5;

    auto extentX = bounds.posMax.X - bounds.posMin.X;
    auto extentY = bounds.posMax.Y - bounds.posMin.Y;
    auto extentZ = bounds.posMax.Z - bounds.posMin.Z;

    first_node.halfSize = std::max(std::max(extentX, extentY), extentZ) * 0.5 * SAFETY_FACTOR;

    if (first_node.halfSize == 0)
        first_node.halfSize += 1;

    first_node.begin = 0;
    first_node.depth = 0;
    first_node.count = nb_bodies;
    this->_nodes.push_back(first_node);
}

void physics::Octree::clear()
{
    this->_nodes.clear();
    this->_permutations.clear();
    this->_buffer.clear();
}
