#pragma once

#include <entt/entt.hpp>
#include <entt/signal/fwd.hpp>
#include "components/GravityCache.hpp"
#include "components/kinematics/Position.hpp"
#include "components/properties/Mass.hpp"
#include "components/solver/ForceAccumulator.hpp"

namespace physics::forces {
    class Gravity {
        public:
            /**
             * @brief Apply gravitational forces between all pairs of entities with Position and Mass components.
             *
             * @param registry The entity registry containing the physics components.
             * @param dt The time step to advance the simulation.
             */
            static void apply(entt::registry& registry, entt::dispatcher& dispatcher, double dt);

            static components::ScalarMass computeScalarMass(const components::Mass& mass);

        private:
            static void _initVectors(size_t estimatedCount);

            static void _computeGravity(size_t count);

            static void _accumulateForce(entt::registry& registry, size_t count);

            static components::Displacement computeDisplacement(const components::Position& posA,
                                                                const components::Position& posB);

            static components::InverseDistance computeInverseDistance(const components::Displacement& disp);

            static double inverseDistance(double distance);

            static constexpr double G = 6.67430e-20; // Gravitational constant
            static constexpr double EPSILON = 1e-6; // Small value to prevent division by zero

            inline static std::vector<entt::entity> _entities;
            inline static std::vector<double> _posX, _posY, _posZ, _mass;
            inline static std::vector<double> _outForceX, _outForceY, _outForceZ;
    }; // namespace Gravity
} // namespace physics::forces
