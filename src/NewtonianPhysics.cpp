#include "NewtonianPhysics.hpp"
#include <entt/signal/fwd.hpp>
#include "components/acceleration.hpp"
#include "components/kinematics/Acceleration.hpp"
#include "components/kinematics/Position.hpp"
#include "components/kinematics/Velocity.hpp"
#include "components/mass.hpp"
#include "components/position.hpp"
#include "components/properties/Mass.hpp"
#include "components/solver/ForceAccumulator.hpp"
#include "components/solver/PreviousAcceleration.hpp"
#include "components/velocity.hpp"
#include "forces/Gravity.hpp"
#include "integration/Verlet.hpp"

//? Public methods

void physics::NewtonianPhysics::init(entt::registry& registry, entt::dispatcher& dispatcher)
{
    physics::NewtonianPhysics::syncIn(registry);

    auto view = registry.view<::common::components::Mass, ::common::components::Position>();

    for (auto entity : view) {
        const auto& mass = view.get<::common::components::Mass>(entity);

        registry.emplace_or_replace<components::ScalarMass>(
            entity, physics::forces::Gravity::computeScalarMass({mass.mantissa, mass.exponent}));
        registry.emplace_or_replace<components::ForceAccumulator>(entity);
        registry.emplace_or_replace<components::PreviousAcceleration>(entity);
    }
}

void physics::NewtonianPhysics::update(entt::registry& registry, entt::dispatcher& dispatcher, double dt)
{
    integration::Verlet::preIntegrate(registry, dt);
    forces::Gravity::apply(registry, dispatcher, dt);
    integration::Verlet::postIntegrate(registry, dt);
}

void physics::NewtonianPhysics::shutdown(entt::registry& registry)
{
    registry.clear<physics::components::PositionX>();
    registry.clear<physics::components::PositionY>();
    registry.clear<physics::components::PositionZ>();
    registry.clear<physics::components::Velocity>();
    registry.clear<physics::components::Acceleration>();
    registry.clear<physics::components::Mass>();
    registry.clear<physics::components::ForceAccumulator>();
}

//? Private methods

void physics::NewtonianPhysics::syncIn(entt::registry& registry)
{
    physics::NewtonianPhysics::syncPositionToPhysics(registry);
    physics::NewtonianPhysics::syncVelocityToPhysics(registry);
    physics::NewtonianPhysics::syncAccelerationToPhysics(registry);
    physics::NewtonianPhysics::syncMassToPhysics(registry);
}

void physics::NewtonianPhysics::syncOut(entt::registry& registry)
{
    physics::NewtonianPhysics::syncPositionToCore(registry);
    physics::NewtonianPhysics::syncVelocityToCore(registry);
    physics::NewtonianPhysics::syncAccelerationToCore(registry);
}

void physics::NewtonianPhysics::syncPositionToPhysics(entt::registry& registry)
{
    auto view = registry.view<::common::components::Position>();

    for (auto entity : view) {
        const auto& position = view.get<::common::components::Position>(entity);

        auto& physicsPositionX = registry.get_or_emplace<physics::components::PositionX>(entity);
        physicsPositionX.value = position.x;
        auto& physicsPositionY = registry.get_or_emplace<physics::components::PositionY>(entity);
        physicsPositionY.value = position.y;
        auto& physicsPositionZ = registry.get_or_emplace<physics::components::PositionZ>(entity);
        physicsPositionZ.value = position.z;
    }
}

void physics::NewtonianPhysics::syncVelocityToPhysics(entt::registry& registry)
{
    auto view = registry.view<::common::components::Velocity>();

    for (auto entity : view) {
        const auto& velocity = view.get<::common::components::Velocity>(entity);
        auto& physicsVelocity = registry.get_or_emplace<physics::components::Velocity>(entity);
        physicsVelocity = {velocity.x, velocity.y, velocity.z};
    }
}

void physics::NewtonianPhysics::syncAccelerationToPhysics(entt::registry& registry)
{
    auto view = registry.view<::common::components::Acceleration>();

    for (auto entity : view) {
        const auto& acceleration = view.get<::common::components::Acceleration>(entity);
        auto& physicsAcceleration = registry.get_or_emplace<physics::components::Acceleration>(entity);
        physicsAcceleration = {acceleration.x, acceleration.y, acceleration.z};
    }
}

void physics::NewtonianPhysics::syncMassToPhysics(entt::registry& registry)
{
    auto view = registry.view<::common::components::Mass>();

    for (auto entity : view) {
        const auto& mass = view.get<::common::components::Mass>(entity);
        auto& physicsMass = registry.get_or_emplace<physics::components::Mass>(entity);
        physicsMass = {mass.mantissa, mass.exponent};
    }
}

void physics::NewtonianPhysics::syncPositionToCore(entt::registry& registry)
{
    auto view =
        registry.view<physics::components::PositionX, physics::components::PositionY, physics::components::PositionZ>();

    for (auto entity : view) {
        const auto& positionX = view.get<physics::components::PositionX>(entity);
        const auto& positionY = view.get<physics::components::PositionY>(entity);
        const auto& positionZ = view.get<physics::components::PositionZ>(entity);

        auto& corePosition = registry.get_or_emplace<::common::components::Position>(entity);
        corePosition = {positionX.value, positionY.value, positionZ.value};
    }
}

void physics::NewtonianPhysics::syncVelocityToCore(entt::registry& registry)
{
    auto view = registry.view<physics::components::Velocity>();

    for (auto entity : view) {
        const auto& velocity = view.get<physics::components::Velocity>(entity);
        auto& coreVelocity = registry.get_or_emplace<::common::components::Velocity>(entity);
        coreVelocity = {velocity.x, velocity.y, velocity.z};
    }
}

void physics::NewtonianPhysics::syncAccelerationToCore(entt::registry& registry)
{
    auto view = registry.view<physics::components::Acceleration>();

    for (auto entity : view) {
        const auto& acceleration = view.get<physics::components::Acceleration>(entity);
        auto& coreAcceleration = registry.get_or_emplace<::common::components::Acceleration>(entity);
        coreAcceleration = {acceleration.x, acceleration.y, acceleration.z};
    }
}
