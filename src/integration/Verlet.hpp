#pragma once

#include <types/World.hpp>

namespace physics::integration {
    class Verlet {
        public:
            static void preIntegrate(common::WorldState& world, double dt);
            static void postIntegrate(common::WorldState& world, double dt);

        private:
    };
} // namespace physics::integration
