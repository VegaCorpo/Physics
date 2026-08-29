#include "Octree.hpp"
#include <cstdint>
#include "components/NewtonianState.hpp"
#include "utils/utils.hpp"

void physics::Octree::build(const physics::NewtonianState& state)
{
    Bounds bounds;
    std::uint32_t nb_bodies;
    Node first_node;

    this->clear();

    for (std::uint32_t i = 0; i < state.size(); i += 1) {
        this->_permutations.push_back(i);
    }
    nb_bodies = this->_permutations.size();

    if (nb_bodies == 0) {
        return;
    }

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
}
