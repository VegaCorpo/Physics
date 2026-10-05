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

    void computeOctantStarts(const physics::Node& parent, const std::uint32_t (&bodiesPerOctant)[physics::OCTANT_COUNT],
                             std::uint32_t (&octantStarts)[physics::OCTANT_COUNT])
    {
        octantStarts[0] = parent.begin;
        for (std::uint32_t octant = 1; octant < physics::OCTANT_COUNT; octant += 1) {
            octantStarts[octant] = octantStarts[octant - 1] + bodiesPerOctant[octant - 1];
        }
    }
} // namespace

void physics::Octree::build(const physics::NewtonianState& state)
{
    std::vector<std::uint32_t> nodesToSubdivide;

    this->clear();

    if (this->_initializeOctree(state) != OctreeState::OK)
        return;

    nodesToSubdivide.push_back(0);
    while (!nodesToSubdivide.empty()) {
        const std::uint32_t nodeIndex = nodesToSubdivide.back();
        nodesToSubdivide.pop_back();

        if (this->_nodes[nodeIndex].count <= LEAF_CAPACITY || this->_nodes[nodeIndex].depth >= MAX_DEPTH) {
            continue;
        }
        this->_subdivide(state, nodeIndex);

        const std::uint32_t firstChild = this->_nodes[nodeIndex].firstChild;

        for (std::uint32_t octant = 0; octant < OCTANT_COUNT; octant += 1) {
            if (this->_nodes[firstChild + octant].count > 0) {
                nodesToSubdivide.push_back(firstChild + octant);
            }
        }
    }
}

void physics::Octree::_subdivide(const physics::NewtonianState& state, std::uint32_t nodeIndex)
{
    const Node parent = this->_nodes[nodeIndex];

    std::uint32_t bodiesPerOctant[OCTANT_COUNT] = {};
    std::uint32_t octantStarts[OCTANT_COUNT] = {};

    this->_countBodiesPerOctant(state, parent, bodiesPerOctant);
    computeOctantStarts(parent, bodiesPerOctant, octantStarts);
    this->_sortBodiesByOctant(state, parent, octantStarts);

    const std::uint32_t firstChild = this->_createChildren(parent, octantStarts, bodiesPerOctant);

    this->_nodes[nodeIndex].firstChild = firstChild;
}

void physics::Octree::_countBodiesPerOctant(const physics::NewtonianState& state, const Node& parent,
                                            std::uint32_t (&bodiesPerOctant)[OCTANT_COUNT])
{
    const std::uint32_t slotsEnd = parent.begin + parent.count;

    for (std::uint32_t slot = parent.begin; slot < slotsEnd; slot += 1) {
        const std::uint32_t bodyIndex = this->_permutations[slot];
        const std::uint32_t octant =
            getOctant(parent, state.posX[bodyIndex], state.posY[bodyIndex], state.posZ[bodyIndex]);

        bodiesPerOctant[octant] += 1;
    }
}

void physics::Octree::_sortBodiesByOctant(const physics::NewtonianState& state, const Node& parent,
                                          const std::uint32_t (&octantStarts)[OCTANT_COUNT])
{
    const std::uint32_t slotsEnd = parent.begin + parent.count;

    std::uint32_t nextFreeSlot[OCTANT_COUNT] = {};

    std::copy(std::begin(octantStarts), std::end(octantStarts), std::begin(nextFreeSlot));
    for (std::uint32_t slot = parent.begin; slot < slotsEnd; slot += 1) {
        const std::uint32_t bodyIndex = this->_permutations[slot];
        const std::uint32_t octant =
            getOctant(parent, state.posX[bodyIndex], state.posY[bodyIndex], state.posZ[bodyIndex]);

        this->_buffer[nextFreeSlot[octant]] = bodyIndex;
        nextFreeSlot[octant] += 1;
    }
    std::copy(this->_buffer.begin() + static_cast<std::ptrdiff_t>(parent.begin),
              this->_buffer.begin() + static_cast<std::ptrdiff_t>(slotsEnd),
              this->_permutations.begin() + static_cast<std::ptrdiff_t>(parent.begin));
}

std::uint32_t physics::Octree::_createChildren(const Node& parent, const std::uint32_t (&octantStarts)[OCTANT_COUNT],
                                               const std::uint32_t (&bodiesPerOctant)[OCTANT_COUNT])
{
    auto firstChild = static_cast<std::uint32_t>(this->_nodes.size());
    const double childHalfSize = parent.halfSize * 0.5;

    for (std::uint32_t octant = 0; octant < OCTANT_COUNT; octant += 1) {
        Node child;

        child.centerX = parent.centerX + ((octant & 1u) ? childHalfSize : -childHalfSize);
        child.centerY = parent.centerY + ((octant & 2u) ? childHalfSize : -childHalfSize);
        child.centerZ = parent.centerZ + ((octant & 4u) ? childHalfSize : -childHalfSize);
        child.halfSize = childHalfSize;
        child.firstChild = INVALID_NODE;
        child.begin = octantStarts[octant];
        child.count = bodiesPerOctant[octant];
        child.depth = parent.depth + 1;

        this->_nodes.push_back(child);
    }
    return firstChild;
}

physics::OctreeState physics::Octree::_initializeOctree(const NewtonianState& state)
{
    std::uint32_t nbBodies;

    for (std::uint32_t bodyIndex = 0; bodyIndex < state.size(); bodyIndex += 1) {
        this->_permutations.push_back(bodyIndex);
    }

    nbBodies = this->_permutations.size();
    if (nbBodies == 0) {
        return OctreeState::NO_BODY;
    }
    this->_buffer.resize(nbBodies);
    this->_initializeFirstNode(state, nbBodies);
    return OctreeState::OK;
}

void physics::Octree::_initializeFirstNode(const physics::NewtonianState& state, std::uint32_t nbBodies)
{
    Node firstNode;
    Bounds bounds;

    for (auto bodyIndex : this->_permutations) {
        bounds.posMin.X = std::min(bounds.posMin.X, state.posX[bodyIndex]);
        bounds.posMin.Y = std::min(bounds.posMin.Y, state.posY[bodyIndex]);
        bounds.posMin.Z = std::min(bounds.posMin.Z, state.posZ[bodyIndex]);

        bounds.posMax.X = std::max(bounds.posMax.X, state.posX[bodyIndex]);
        bounds.posMax.Y = std::max(bounds.posMax.Y, state.posY[bodyIndex]);
        bounds.posMax.Z = std::max(bounds.posMax.Z, state.posZ[bodyIndex]);
    }

    firstNode.centerX = (bounds.posMin.X + bounds.posMax.X) * 0.5;
    firstNode.centerY = (bounds.posMin.Y + bounds.posMax.Y) * 0.5;
    firstNode.centerZ = (bounds.posMin.Z + bounds.posMax.Z) * 0.5;

    auto extentX = bounds.posMax.X - bounds.posMin.X;
    auto extentY = bounds.posMax.Y - bounds.posMin.Y;
    auto extentZ = bounds.posMax.Z - bounds.posMin.Z;

    firstNode.halfSize = std::max(std::max(extentX, extentY), extentZ) * 0.5 * SAFETY_FACTOR;

    if (firstNode.halfSize == 0)
        firstNode.halfSize += 1;

    firstNode.begin = 0;
    firstNode.depth = 0;
    firstNode.count = nbBodies;
    this->_nodes.push_back(firstNode);
}

void physics::Octree::clear()
{
    this->_nodes.clear();
    this->_permutations.clear();
    this->_buffer.clear();
}
