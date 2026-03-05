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
             * @brief Initialize the physics engine and synchronize existing entities.
             *
             * @param registry The entity registry containing the entities to initialize.
             */
            static void init(entt::registry& registry);

            /**
             * @brief Advance the physics simulation by one time step.
             *
             * @param registry The entity registry containing the physics components.
             * @param dt The time step in seconds.
             */
            static void update(entt::registry& registry, double dt);

            /**
             * @brief Shutdown the physics engine and remove all private physics components from the registry.
             *
             * @param registry The entity registry to clean up.
             */
            static void shutdown(entt::registry& registry);

            /**
             * @brief Copy Core components into private Physics components before the simulation step.
             *
             * @param registry The entity registry to read from.
             */
            static void syncIn(entt::registry& registry);

            /**
             * @brief Copy computed Physics components back into Core components after the simulation step.
             *
             * @param registry The entity registry to write to.
             */
            static void syncOut(entt::registry& registry);

            /**
             * @brief Get the name of the physics engine.
             *
             * @return The name of the physics engine.
             */
            static std::string getName() { return "NewtonianPhysics"; }

        private:
            static void prepareStep(entt::registry& registry);

            // --- Sync In helpers : copy Core components into private Physics components ---

            static void syncPositionToPhysics(entt::registry& registry);
            static void syncVelocityToPhysics(entt::registry& registry);
            static void syncAccelerationToPhysics(entt::registry& registry);
            static void syncMassToPhysics(entt::registry& registry);

            // --- Sync Out helpers : copy computed Physics components back into Core components ---

            static void syncPositionToCore(entt::registry& registry);
            static void syncVelocityToCore(entt::registry& registry);
            static void syncAccelerationToCore(entt::registry& registry);
            static void syncMassToCore(entt::registry& registry);
    };
} // namespace physics
