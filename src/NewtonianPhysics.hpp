#pragma once

#include <components/acceleration.hpp>
#include <components/mass.hpp>
#include <components/position.hpp>
#include <components/velocity.hpp>

#include <entt/entt.hpp>
#include <entt/signal/fwd.hpp>

#include <interfaces/IPhysicsEngine.hpp>

namespace physics {
    class NewtonianPhysics : public common::IPhysicsEngine {
        public:
            /**
             * @brief Initialize the physics engine and synchronize existing entities.
             *
             * @param registry The entity registry containing the entities to initialize.
             * @param dispatcher to delay function executions.
             */
            NewtonianPhysics() = default;
            ~NewtonianPhysics() = default;

            virtual void init(entt::registry &registry, entt::dispatcher &dispatcher) override;
            /**
             * @brief Advance the physics simulation by one time step.
             *
             * @param registry The entity registry containing the physics components.
             * @param dispatcher to delay function executions.
             * @param dt The time step in seconds.
             */
            void update(entt::registry& registry, entt::dispatcher& dispatcher, double dt) override;

            /**
             * @brief Shutdown the physics engine and remove all private physics components from the registry.
             *
             * @param registry The entity registry to clean up.
             */
            void shutdown(entt::registry& registry) override;

            /**
             * @brief Copy Core components into private Physics components before the simulation step.
             *
             * @param registry The entity registry to read from.
             */
            void syncIn(entt::registry& registry) override;

            /**
             * @brief Copy computed Physics components back into Core components after the simulation step.
             *
             * @param registry The entity registry to write to.
             */
            void syncOut(entt::registry& registry) override;

            /**
             * @brief Get the name of the physics engine.
             *
             * @return The name of the physics engine.
             */
            [[nodiscard]] std::string getName() const override { return "NewtonianPhysics"; }

        private:
            // --- Sync In helpers : copy Core components into private Physics components ---

            void _syncPositionToPhysics(entt::registry& registry);
            void _syncVelocityToPhysics(entt::registry& registry);
            void _syncAccelerationToPhysics(entt::registry& registry);
            void _syncMassToPhysics(entt::registry& registry);

            // --- Sync Out helpers : copy computed Physics components back into Core components ---

            void _syncPositionToCore(entt::registry& registry) const;
            void _syncVelocityToCore(entt::registry& registry) const;
            void _syncAccelerationToCore(entt::registry& registry) const;
            void _syncMassToCore(entt::registry& registry) const;
    };
} // namespace physics
