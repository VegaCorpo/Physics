#pragma once

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
    };
} // namespace physics
