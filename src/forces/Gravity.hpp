#pragma once

#include <entt/entt.hpp>
#include "../components/kinematics/Position.hpp"
#include "../components/properties/Mass.hpp"
#include "../components/solver/ForceAccumulator.hpp"

namespace physics {
    namespace forces {
        namespace Gravity {
            static constexpr double G = 6.67430e-11; // Gravitational constant
            static constexpr double EPSILON = 1e-6; // Small value to prevent division by zero

            /**
             * @brief Apply gravitational forces between all pairs of entities with Position and Mass components.
             *
             * @param registry The entity registry containing the physics components.
             * @param dt The time step to advance the simulation.
             */
            inline void apply(entt::registry& registry, double /*dt*/)
            {
                auto view = registry.view<physics::components::Position, physics::components::Mass,
                                          physics::components::ForceAccumulator>();

                for (auto [entityA, posA, massA, forceA] : view.each()) {
                    double m1 = massA.mantissa * std::pow(10.0, massA.exponent);
                    for (auto [entityB, posB, massB, forceB] : view.each()) {
                        if (entityA >= entityB) {
                            continue;
                        }

                        double dx = posB.x - posA.x;
                        double dy = posB.y - posA.y;
                        double dz = posB.z - posA.z;
                        double dSqrt = dx * dx + dy * dy + dz * dz + EPSILON * EPSILON;
                        double distance = std::sqrt(dSqrt);

                        double m2 = massB.mantissa * std::pow(10.0, massB.exponent);
                        double forceMagnitude = G * m1 * m2 / dSqrt;
                        double fx = forceMagnitude * dx / distance;
                        double fy = forceMagnitude * dy / distance;
                        double fz = forceMagnitude * dz / distance;

                        forceA.x += fx;
                        forceA.y += fy;
                        forceA.z += fz;
                        forceB.x -= fx;
                        forceB.y -= fy;
                        forceB.z -= fz;
                    }
                }
            }
        }; // namespace Gravity
    } // namespace forces
} // namespace physics
