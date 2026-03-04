#include "NewtonianPhysics.hpp"
#include "forces/Gravity.hpp"
#include "components/kinematics/Acceleration.hpp"
#include "components/kinematics/Position.hpp"
#include "components/kinematics/Velocity.hpp"
#include "components/properties/Mass.hpp"
#include "components/solver/ForceAccumulator.hpp"
#include "components/solver/PreviousPosition.hpp"

//? Public methods

void physics::NewtonianPhysics::init(entt::registry& registry)
{
    syncIn(registry);
}

void physics::NewtonianPhysics::update(entt::registry& registry, double dt)
{
    prepareStep(registry);
    // forces::Gravity::accumulate(registry, static_cast<float>(dt));
    // Integrator
    syncOut(registry);
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
    auto view = registry.view<const components::Position, const components::Velocity, const components::Acceleration,
                              const components::Mass>();

    for (auto [entity, position, velocity, acceleration, mass] : view.each()) {
        if (!registry.all_of<physics::components::Position>(entity)) {
            registry.emplace<physics::components::Position>(entity, 0.0, 0.0, 0.0);
            registry.emplace<physics::components::Velocity>(entity, 0.0, 0.0, 0.0);
            registry.emplace<physics::components::Acceleration>(entity, 0.0, 0.0, 0.0);
            registry.emplace<physics::components::Mass>(entity, 1.0, 0);
            registry.emplace<physics::components::ForceAccumulator>(entity, 0.0, 0.0, 0.0);
            registry.emplace<physics::components::PreviousPosition>(entity, 0.0, 0.0, 0.0);
        }
        registry.get<physics::components::Position>(entity) = {position.x, position.y, position.z};
        registry.get<physics::components::Velocity>(entity) = {velocity.x, velocity.y, velocity.z};
        registry.get<physics::components::Acceleration>(entity) = {acceleration.x, acceleration.y, acceleration.z};
        registry.get<physics::components::Mass>(entity) = {mass.mantissa, mass.exponent};
    }
}

void physics::NewtonianPhysics::syncOut(entt::registry& registry)
{
    auto view = registry.view<const physics::components::Position, const physics::components::Velocity,
                              const physics::components::Acceleration>();

    for (auto [entity, position, velocity, acceleration] : view.each()) {
        if (registry.all_of<components::Position>(entity)) {
            registry.get<components::Position>(entity) = {position.x, position.y, position.z};
            registry.get<components::Velocity>(entity) = {velocity.x, velocity.y, velocity.z};
            registry.get<components::Acceleration>(entity) = {acceleration.x, acceleration.y, acceleration.z};
        }
    }
}

void physics::NewtonianPhysics::prepareStep(entt::registry& registry)
{
    auto view = registry.view<physics::components::ForceAccumulator>();

    for (auto [entity, forceAccumulator] : view.each()) {
        forceAccumulator = {0.0, 0.0, 0.0};
    }
}
