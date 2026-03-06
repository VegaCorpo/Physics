#include "Verlet.hpp"
#include "../components/GravityCache.hpp"
#include "../components/kinematics/Acceleration.hpp"
#include "../components/kinematics/Position.hpp"
#include "../components/kinematics/Velocity.hpp"
#include "../components/solver/ForceAccumulator.hpp"
#include "../components/solver/PreviousAcceleration.hpp"

//? Public methods

void physics::integration::Verlet::integrate(entt::registry& registry, double dt)
{
    physics::integration::Verlet::_computeAcceleration(registry);
    physics::integration::Verlet::_updatePositionAndVelocity(registry, dt);
}

//? Private methods

void physics::integration::Verlet::_computeAcceleration(entt::registry& registry)
{
    auto view = registry.view<physics::components::ForceAccumulator, physics::components::ScalarMass,
                              physics::components::Acceleration, physics::components::PreviousAcceleration,
                              physics::components::Position>();

    for (auto [entity, force, mass, acc, prevAcc, pos] : view.each()) {
        prevAcc = {acc.x, acc.y, acc.z};

        // Compute acceleration: a = F / m
        acc.x = force.x / mass.value;
        acc.y = force.y / mass.value;
        acc.z = force.z / mass.value;
    }
}

void physics::integration::Verlet::_updatePositionAndVelocity(entt::registry& registry, double dt)
{
    auto view = registry.view<physics::components::Position, physics::components::Velocity,
                              physics::components::Acceleration, physics::components::PreviousAcceleration>();

    for (auto [entity, pos, vel, acc, prevAcc] : view.each()) {
        // Update position: x(t + dt) = x(t) + v(t) * dt + 0.5 * a(t) * dt^2
        pos.x += vel.x * dt + 0.5 * prevAcc.x * dt * dt;
        pos.y += vel.y * dt + 0.5 * prevAcc.y * dt * dt;
        pos.z += vel.z * dt + 0.5 * prevAcc.z * dt * dt;

        // Update velocity: v(t + dt) = v(t) + 0.5 * (a(t) + a(t + dt)) * dt
        vel.x += 0.5 * (prevAcc.x + acc.x) * dt;
        vel.y += 0.5 * (prevAcc.y + acc.y) * dt;
        vel.z += 0.5 * (prevAcc.z + acc.z) * dt;
    }
}
