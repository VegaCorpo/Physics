#include "NewtonianPhysics.hpp"
#include "components/kinematics/Acceleration.hpp"
#include "components/kinematics/Position.hpp"
#include "components/kinematics/Velocity.hpp"
#include "components/properties/Mass.hpp"
#include "components/solver/ForceAccumulator.hpp"
#include "components/solver/PreviousPosition.hpp"
#include "forces/Gravity.hpp"

//? Public methods

void physics::NewtonianPhysics::init(entt::registry& /*registry*/)
{
    // No specific initialization needed for now
}

void physics::NewtonianPhysics::update(entt::registry& registry, double /*dt*/)
{
    physics::NewtonianPhysics::prepareStep(registry);
    // forces::Gravity::apply(registry, static_cast<float>(dt));
    // Integrator
    physics::NewtonianPhysics::syncOut(registry);
}

void physics::NewtonianPhysics::shutdown(entt::registry& registry)
{
    registry.clear<physics::components::Position>();
    registry.clear<physics::components::Velocity>();
    registry.clear<physics::components::Acceleration>();
    registry.clear<physics::components::Mass>();
    registry.clear<physics::components::ForceAccumulator>();
    registry.clear<physics::components::PreviousPosition>();
}

//? Private methods

void physics::NewtonianPhysics::syncIn(entt::registry& registry)
{
    syncPositionToPhysics(registry);
    syncVelocityToPhysics(registry);
    syncAccelerationToPhysics(registry);
    syncMassToPhysics(registry);
}

void physics::NewtonianPhysics::syncOut(entt::registry& registry)
{
    syncPositionToCore(registry);
    syncVelocityToCore(registry);
    syncAccelerationToCore(registry);
}

void physics::NewtonianPhysics::prepareStep(entt::registry& registry)
{
    auto view = registry.view<physics::components::ForceAccumulator>();

    for (auto [entity, forceAccumulator] : view.each()) {
        forceAccumulator = {};
    }
}

void physics::NewtonianPhysics::syncPositionToPhysics(entt::registry& registry)
{
    auto view = registry.view<components::Position>();

    for (auto [entity, position] : view.each()) {
        auto& physicsPosition = registry.get_or_emplace<physics::components::Position>(entity);
        physicsPosition = {position.x, position.y, position.z};
    }
}

void physics::NewtonianPhysics::syncVelocityToPhysics(entt::registry& registry)
{
    auto view = registry.view<components::Velocity>();

    for (auto [entity, velocity] : view.each()) {
        auto& physicsVelocity = registry.get_or_emplace<physics::components::Velocity>(entity);
        physicsVelocity = {velocity.x, velocity.y, velocity.z};
    }
}

void physics::NewtonianPhysics::syncAccelerationToPhysics(entt::registry& registry)
{
    auto view = registry.view<components::Acceleration>();

    for (auto [entity, acceleration] : view.each()) {
        auto& physicsAcceleration = registry.get_or_emplace<physics::components::Acceleration>(entity);
        physicsAcceleration = {acceleration.x, acceleration.y, acceleration.z};
    }
}

void physics::NewtonianPhysics::syncMassToPhysics(entt::registry& registry)
{
    auto view = registry.view<components::Mass>();

    for (auto [entity, mass] : view.each()) {
        auto& physicsMass = registry.get_or_emplace<physics::components::Mass>(entity);
        physicsMass = {mass.mantissa, mass.exponent};
    }
}

void physics::NewtonianPhysics::syncPositionToCore(entt::registry& registry)
{
    auto view = registry.view<const physics::components::Position>();

    for (auto [entity, position] : view.each()) {
        auto& corePosition = registry.get_or_emplace<components::Position>(entity);
        corePosition = {position.x, position.y, position.z};
    }
}

void physics::NewtonianPhysics::syncVelocityToCore(entt::registry& registry)
{
    auto view = registry.view<const physics::components::Velocity>();

    for (auto [entity, velocity] : view.each()) {
        auto& coreVelocity = registry.get_or_emplace<components::Velocity>(entity);
        coreVelocity = {velocity.x, velocity.y, velocity.z};
    }
}

void physics::NewtonianPhysics::syncAccelerationToCore(entt::registry& registry)
{
    auto view = registry.view<const physics::components::Acceleration>();

    for (auto [entity, acceleration] : view.each()) {
        auto& coreAcceleration = registry.get_or_emplace<components::Acceleration>(entity);
        coreAcceleration = {acceleration.x, acceleration.y, acceleration.z};
    }
}
