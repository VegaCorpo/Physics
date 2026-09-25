#pragma once

#include <interfaces/IPhysicsEngine.hpp>
#include "components/NewtonianState.hpp"
#include "spatial/Octree.hpp"
#include "types/World.hpp"

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
            ~NewtonianPhysics() override = default;

            void init(common::SpecificDataPhysics world) override;
            /**
             * @brief Advance the physics simulation by one time step.
             *
             * @param registry The entity registry containing the physics components.
             * @param dispatcher to delay function executions.
             * @param dt The time step in seconds.
             */
            void update(double dt) override;

            /**
             * @brief Shutdown the physics engine and remove all private physics components from the registry.
             *
             * @param registry The entity registry to clean up.
             */
            void shutdown() override;

            /**
             * @brief Copy Core components into private Physics components before the simulation step.
             *
             * @param registry The entity registry to read from.
             */
            void syncIn(common::SpecificDataPhysics world) override;

            /**
             * @brief Copy computed Physics components back into Core components after the simulation step.
             *
             * @param registry The entity registry to write to.
             */
            common::WorldState publish() override;

            /**
             * @brief Get the name of the physics engine.
             *
             * @return The name of the physics engine.
             */
            [[nodiscard]] std::string getName() const override { return "NewtonianPhysics"; }

        private:
            common::SpecificDataPhysics _world_state;
            NewtonianState _newtonian_state;

            Octree _octree;
    };
} // namespace physics
