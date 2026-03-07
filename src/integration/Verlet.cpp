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
                              physics::components::Acceleration, physics::components::PreviousAcceleration>();

    for (auto [entity, force, mass, acc, prevAcc] : view.each()) {
        prevAcc = {acc.x, acc.y, acc.z};
        if (mass.value == 0.0) {
            acc = {0.0, 0.0, 0.0};
            continue;
        }

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

    auto halfDtSquared = 0.5 * dt * dt;
    auto halfDt = 0.5 * dt;

    for (auto [entity, pos, vel, acc, prevAcc] : view.each()) {
        // Update position: x(t + dt) = x(t) + v(t) * dt + 0.5 * a(t) * dt^2
        pos.x += vel.x * dt + halfDtSquared * prevAcc.x;
        pos.y += vel.y * dt + halfDtSquared * prevAcc.y;
        pos.z += vel.z * dt + halfDtSquared * prevAcc.z;

        // Update velocity: v(t + dt) = v(t) + 0.5 * (a(t) + a(t + dt)) * dt
        vel.x += (prevAcc.x + acc.x) * halfDt;
        vel.y += (prevAcc.y + acc.y) * halfDt;
        vel.z += (prevAcc.z + acc.z) * halfDt;
    }
}
