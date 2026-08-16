#pragma once

#include <types/World.hpp>
#include "components/gravity_cache/GravityCache.hpp"
#include "components/NewtonianState.hpp"

namespace physics::forces {

    constexpr double G = 6.67430e-20; // Gravitational constant
    constexpr double EPSILON = 1e-6; // Small value to prevent division by zero
    constexpr double EPSILON2 = EPSILON * EPSILON;

    class Gravity {
        public:
            static void apply(NewtonianState& state, double dt);

            static components::ScalarMass computeScalarMass(const common::components::Mass& mass);

        private:
            static void _computeGravity(NewtonianState& state);
            static inline void _accumulate();

            static components::InverseDistance computeInverseDistance(const components::Displacement& disp);
    };
} // namespace physics::forces
