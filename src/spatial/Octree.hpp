#pragma once

#include <cstdint>
#include <limits>
#include <vector>
#include "components/NewtonianState.hpp"

namespace physics {

    constexpr uint8_t LEAF_CAPACITY = 8;
    constexpr uint8_t MAX_DEPTH = 20;
    constexpr std::uint32_t INVALID_NODE = std::numeric_limits<std::uint32_t>::max();
    constexpr double SAFETY_FACTOR = 1.001;

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

        private:
            std::vector<Node> _nodes;
            std::vector<std::uint32_t> _permutations;
            std::vector<std::uint32_t> _buffer;

            void _subdivide(const NewtonianState& state, std::uint32_t index);
    };
} // namespace physics
