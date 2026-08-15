#pragma once

#include <boost/align/aligned_allocator.hpp>
#include <experimental/simd>
#include <types/World.hpp>
#include "components/GravityCache.hpp"
#include "components/properties/Mass.hpp"

namespace physics::forces {

    constexpr double G = 6.67430e-20; // Gravitational constant
    constexpr double EPSILON = 1e-6; // Small value to prevent division by zero
    constexpr double EPSILON2 = EPSILON * EPSILON;

    class Gravity {
        public:
            /**
             * @brief Apply gravitational forces between all pairs of entities with Position and Mass components.
             *
             * @param registry The entity registry containing the physics components.
             * @param dt The time step to advance the simulation.
             */
            static void apply(common::WorldState& world, double dt);

            static components::ScalarMass computeScalarMass(const components::Mass& mass);

        private:
            static void _computeGravity(common::WorldState& world, size_t count);

            static components::InverseDistance computeInverseDistance(const components::Displacement& disp);

            static double inverseDistance(double distance);
    }; // namespace Gravity
} // namespace physics::forces
