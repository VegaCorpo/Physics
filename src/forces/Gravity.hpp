#pragma once

#include <entt/entt.hpp>
#include "../components/kinematics/Position.hpp"
#include "../components/properties/Mass.hpp"
#include "../components/solver/ForceAccumulator.hpp"

namespace physics {
    namespace forces {
        class Gravity {
            public:
                /**
                 * @brief Apply gravitational forces between all pairs of entities with Position and Mass components.
                 *
                 * @param registry The entity registry containing the physics components.
                 * @param dt The time step to advance the simulation.
                 */
                static void apply(entt::registry& registry, double dt);

            private:
                static constexpr double G = 6.67430e-11; // Gravitational constant
                static constexpr double EPSILON = 1e-6; // Small value to prevent division by zero

                static void applyPairwiseGravity(const components::Position& posA, const components::Position& posB,
                                                 double m1, double m2, components::ForceAccumulator& forceA,
                                                 components::ForceAccumulator& forceB);
                static double computeMassValue(const physics::components::Mass& mass);
                static double computeForceMagnitude(double m1, double m2, double invDistanceCubed);
        }; // namespace Gravity
    } // namespace forces
} // namespace physics
