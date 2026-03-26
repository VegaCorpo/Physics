#include "Verlet.hpp"
#include "../components/kinematics/Position.hpp"
#include "../components/kinematics/Velocity.hpp"
#include "../components/properties/Mass.hpp"
#include "../components/solver/ForceAccumulator.hpp"
#include "components/GravityCache.hpp"

void physics::integration::Verlet::preIntegrate(entt::registry& registry, double dt)
{
    auto view = registry.view<physics::components::PositionX, physics::components::PositionY,
                              physics::components::PositionZ, physics::components::Velocity,
                              physics::components::ForceAccumulator, physics::components::ScalarMass>();

    const double halfDt = 0.5 * dt;

    for (auto entity : view) {
        auto& posX = view.get<physics::components::PositionX>(entity);
        auto& posY = view.get<physics::components::PositionY>(entity);
        auto& posZ = view.get<physics::components::PositionZ>(entity);
        auto& vel = view.get<physics::components::Velocity>(entity);
        auto& force = view.get<physics::components::ForceAccumulator>(entity);
        const auto& mass = view.get<physics::components::ScalarMass>(entity);

        if (mass.value == 0.0)
            continue;

        double ax = force.x / mass.value;
        double ay = force.y / mass.value;
        double az = force.z / mass.value;

        vel.x += ax * halfDt;
        vel.y += ay * halfDt;
        vel.z += az * halfDt;

        posX.value += vel.x * dt;
        posY.value += vel.y * dt;
        posZ.value += vel.z * dt;

        force = {0.0, 0.0, 0.0};
    }
}

void physics::integration::Verlet::postIntegrate(entt::registry& registry, double dt)
{
    auto view = registry.view<physics::components::Velocity, physics::components::ForceAccumulator,
                              physics::components::ScalarMass>();

    const double halfDt = 0.5 * dt;

    for (auto entity : view) {
        auto& vel = view.get<physics::components::Velocity>(entity);
        const auto& force = view.get<physics::components::ForceAccumulator>(entity);
        const auto& mass = view.get<physics::components::ScalarMass>(entity);

        if (mass.value == 0.0)
            continue;

        double ax = force.x / mass.value;
        double ay = force.y / mass.value;
        double az = force.z / mass.value;

        vel.x += ax * halfDt;
        vel.y += ay * halfDt;
        vel.z += az * halfDt;
    }
}
