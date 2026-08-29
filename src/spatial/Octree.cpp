#include "Octree.hpp"
#include <cstdint>
#include "components/NewtonianState.hpp"
#include "utils/utils.hpp"

namespace {
    std::uint32_t getOctant(const physics::Node& node, double x, double y, double z)
    {
        return ((x >= node.centerX ? 1u : 0u) | (y >= node.centerY ? 2u : 0u) | (z >= node.centerZ ? 4u : 0u));
    }
} // namespace

void physics::Octree::build(const physics::NewtonianState& state)
{
    std::vector<std::uint32_t> pending;

    this->clear();

    if (this->_initializeOctree(state) != OctreeState::OK)
        return;

    pending.push_back(0);
    while (!pending.empty()) {
        const std::uint32_t index = pending.back();
        pending.pop_back();

        if (this->_nodes[index].count <= LEAF_CAPACITY || this->_nodes[index].depth >= MAX_DEPTH) {
            continue;
        }
        this->_subdivide(state, index);

        const std::uint32_t first_child = this->_nodes[index].first_child;

        for (std::uint32_t octant = 0; octant < LEAF_CAPACITY; octant += 1) {
            if (this->_nodes[first_child + octant].count > 0) {
                pending.push_back(first_child + octant);
            }
        }
    }
}

physics::OctreeState physics::Octree::_initializeOctree(const NewtonianState& state)
{
    std::uint32_t nb_bodies;

    for (std::uint32_t i = 0; i < state.size(); i += 1) {
        this->_permutations.push_back(i);
    }

    nb_bodies = this->_permutations.size();
    if (nb_bodies == 0) {
        return OctreeState::OK;
    }
    this->_buffer.resize(nb_bodies);
    this->_initializeFirstNode(state, nb_bodies);
    return OctreeState::NO_BODY;
}

void physics::Octree::_initializeFirstNode(const physics::NewtonianState& state, std::uint32_t nb_bodies)
{
    Node first_node;
    Bounds bounds;

    for (auto index : this->_permutations) {
        bounds.posMin.X = std::min(bounds.posMin.X, state.posX[index]);
        bounds.posMin.Y = std::min(bounds.posMin.Y, state.posY[index]);
        bounds.posMin.Z = std::min(bounds.posMin.Z, state.posZ[index]);

        bounds.posMax.X = std::max(bounds.posMax.X, state.posX[index]);
        bounds.posMax.Y = std::max(bounds.posMax.Y, state.posY[index]);
        bounds.posMax.Z = std::max(bounds.posMax.Z, state.posZ[index]);
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

void physics::Octree::_subdivide(const physics::NewtonianState& state, std::uint32_t index)
{}

void physics::Octree::clear()
{
    this->_nodes.clear();
    this->_permutations.clear();
    this->_buffer.clear();
}
