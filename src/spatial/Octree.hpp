#pragma once

#include <cstdint>
#include <vector>
#include "components/NewtonianState.hpp"

namespace physics {

    constexpr double SAFETY_FACTOR = 1.001;

    struct Node {
            double centerX;
            double centerY;
            double centerZ;
            double halfSize;
            std::uint32_t first_child;
            std::uint32_t begin;
            std::uint32_t count;
            std::uint32_t depth;
    }; // Size 48

    class Octree {
        public:
            void build(const physics::NewtonianState& state);
            void clear();

        private:
            std::vector<Node> _nodes;
            std::vector<std::uint32_t> _permutations;
    };
} // namespace physics
