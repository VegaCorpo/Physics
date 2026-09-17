#pragma once

#include <cstddef>
#include <cstdint>
#include <tuple>
#include "components/NewtonianState.hpp"
#include "spatial/Octree.hpp"

namespace physics {

    class Collider {
        public:
            std::vector<std::pair<std::uint32_t, std::uint32_t>> check_collisions(NewtonianState& state,
                                                                                  Octree& octree);

        private:
            // std::vector<std::tuple<std::uint32_t>> _check_node_collisions(Octree& octree, std::uint32_t node_index);
            void _visit(const Octree& octree, const NewtonianState& state, std::uint32_t node_index,
                        std::vector<std::pair<uint32_t, uint32_t>>& out);
    };
} // namespace physics
