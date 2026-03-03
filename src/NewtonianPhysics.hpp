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
            void init(entt::registry& registry);
            /**
             * @brief Update the physics simulation by a given time step.
             *
             * @param registry The entity registry containing the physics components.
             * @param dt The time step to advance the simulation by.
             */
            void update(entt::registry& registry, double dt);
            /**
             * @brief Shutdown the physics engine and clean up resources.
             *
             * @param registry The entity registry to clean up physics components from.
             */
            void shutdown(entt::registry& registry);
    };
} // namespace physics
