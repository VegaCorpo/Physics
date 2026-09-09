#pragma once

#include <cstdint>
#include <limits>
#include <vector>
#include "components/NewtonianState.hpp"

namespace physics {

    constexpr std::uint32_t OCTANT_COUNT = 8;
    constexpr uint8_t LEAF_CAPACITY = 8;
    constexpr uint8_t MAX_DEPTH = 20;
    constexpr std::uint32_t INVALID_NODE = std::numeric_limits<std::uint32_t>::max();
    constexpr double SAFETY_FACTOR = 1.001;

    enum class OctreeState {
        OK,
        NO_BODY,
    };

    struct Node {
            double centerX = 0;
            double centerY = 0;
            double centerZ = 0;
            double halfSize = 1;
            std::uint32_t first_child = INVALID_NODE;
            std::uint32_t begin = 0;
            std::uint32_t count = 0;
            std::uint32_t depth = 0;
    }; // Size 48

    class Octree {
        public:
            void build(const physics::NewtonianState& state);
            void clear();

            [[nodiscard]] const std::vector<Node>& nodes() const noexcept { return this->_nodes; }
            [[nodiscard]] const std::vector<std::uint32_t>& permutations() const noexcept
            {
                return this->_permutations;
            }

        private:
            std::vector<Node> _nodes;
            std::vector<std::uint32_t> _permutations;
            std::vector<std::uint32_t> _buffer;

            OctreeState _initializeOctree(const physics::NewtonianState& state);
            void _initializeFirstNode(const physics::NewtonianState& state, std::uint32_t nb_bodies);
            void _subdivide(const NewtonianState& state, std::uint32_t node_index);
            void _countBodiesPerOctant(const NewtonianState& state, const Node& parent,
                                       std::uint32_t (&bodies_per_octant)[OCTANT_COUNT]);
            void _sortBodiesByOctant(const NewtonianState& state, const Node& parent,
                                     const std::uint32_t (&octant_starts)[OCTANT_COUNT]);
            std::uint32_t _createChildren(const Node& parent, const std::uint32_t (&octant_starts)[OCTANT_COUNT],
                                          const std::uint32_t (&bodies_per_octant)[OCTANT_COUNT]);
    };
} // namespace physics
