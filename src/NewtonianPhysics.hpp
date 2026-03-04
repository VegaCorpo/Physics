#pragma once

#include <components/acceleration.hpp>
#include <components/mass.hpp>
#include <components/position.hpp>
#include <components/velocity.hpp>

#include <entt/entt.hpp>

namespace physics {
    class NewtonianPhysics {
        public:
            /**
             * @brief Initialize the newtonian physics engine with the given registry.
             *
             * @param registry The entity registry to use for storing physics components.
             */
            static void init(entt::registry& registry);
            /**
             * @brief Update the physics simulation by a given time step.
             *
             * @param registry The entity registry containing the physics components.
             * @param dt The time step to advance the simulation by.
             */
            static void update(entt::registry& registry, double dt);
            /**
             * @brief Shutdown the physics engine and clean up resources.
             *
             * @param registry The entity registry to clean up physics components from.
             */
            static void shutdown(entt::registry& registry);

            /**
             * @brief Get the name of the physics engine.
             *
             * @return The name of the physics engine.
             */
            static std::string getName() { return "NewtonianPhysics"; }
        private:
            /**
             * @brief Synchronize the physics components from the registry to the physics engine before updating.
             *
             * @param registry The entity registry containing the physics components to synchronize.
             */
            static void syncIn(entt::registry& registry);
            /**
             * @brief Synchronize the physics components from the physics engine back to the registry after updating.
             *
             * @param registry The entity registry to update with the latest physics component data.
             */
            static void syncOut(entt::registry& registry);
            /**
             * @brief Prepare the physics engine for the next simulation step (e.g., clear force accumulators).
             *
             * @param registry The entity registry containing the physics components to prepare.
             */
            static void prepareStep(entt::registry& registry);
    };
} // namespace physics
