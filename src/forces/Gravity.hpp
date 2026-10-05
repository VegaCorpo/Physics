#pragma once

#include <types/World.hpp>
#include "components/gravity_cache/GravityCache.hpp"
#include "components/NewtonianState.hpp"

namespace physics::forces {

    constexpr double G = 6.67430e-20; // Gravitational constant

    class Gravity {
        public:
            static void apply(NewtonianState& state, double dt);

            static components::ScalarMass computeScalarMass(const common::components::Mass& mass);

        private:
            static void _computeGravity(NewtonianState& state);
            static inline void _accumulate();

            static components::InverseDistance computeInverseDistance(const components::Displacement& disp,
                                                                      double epsilon);
    };
} // namespace physics::forces
