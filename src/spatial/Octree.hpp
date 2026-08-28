#pragma once

#include <components/acceleration.hpp>
#include <components/mass.hpp>
#include <components/position.hpp>
#include <components/radius.hpp>
#include <components/velocity.hpp>
#include <cstdint>
#include <limits>
#include <vector>
#include "components/NewtonianState.hpp"

namespace physics {

    struct Min {
            double X = std::numeric_limits<double>::infinity();
            double Y = std::numeric_limits<double>::infinity();
            double Z = std::numeric_limits<double>::infinity();
    }; // Size 24

    struct Max {
            double X = std::numeric_limits<double>::lowest();
            double Y = std::numeric_limits<double>::lowest();
            double Z = std::numeric_limits<double>::lowest();
    }; // Size 24

    struct Bounds {
            Min posMin;
            Max posMax;
    }; // Size 48

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
