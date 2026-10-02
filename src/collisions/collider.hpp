#pragma once

#include <cstdint>
#include "components/NewtonianState.hpp"
#include "spatial/Octree.hpp"

namespace physics {

    class Collider {
        public:
            std::vector<std::pair<std::uint32_t, std::uint32_t>> check_collisions(NewtonianState& state,
                                                                                  Octree& octree);

        private:
            void _visit(const Octree& octree, const NewtonianState& state, std::uint32_t node_index,
                        std::vector<std::pair<uint32_t, uint32_t>>& out);
    };
} // namespace physics
