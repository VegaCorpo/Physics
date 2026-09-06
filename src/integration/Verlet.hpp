#pragma once

#include <types/World.hpp>
#include "components/NewtonianState.hpp"

namespace physics::integration {
    class Verlet {
        public:
            static void preIntegrate(common::SpecificDataPhysics& world, NewtonianState& state, double dt);
            static void postIntegrate(common::SpecificDataPhysics& world, NewtonianState& state, double dt);

        private:
    };
} // namespace physics::integration
