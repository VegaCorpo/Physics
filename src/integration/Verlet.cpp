#include "Verlet.hpp"

void physics::integration::Verlet::preIntegrate(common::WorldState& world, double dt)
{
    const double halfDt = 0.5 * dt;

    // TODO integrate world state
    // for (auto entity : view) {
    //     auto& posX = view.get<physics::components::PositionX>(entity);
    //     auto& posY = view.get<physics::components::PositionY>(entity);
    //     auto& posZ = view.get<physics::components::PositionZ>(entity);
    //     auto& vel = view.get<physics::components::Velocity>(entity);
    //     auto& force = view.get<physics::components::ForceAccumulator>(entity);
    //     const auto& mass = view.get<physics::components::ScalarMass>(entity);
    //
    //     if (mass.value == 0.0)
    //         continue;
    //
    //     double ax = force.x / mass.value;
    //     double ay = force.y / mass.value;
    //     double az = force.z / mass.value;
    //
    //     vel.x += ax * halfDt;
    //     vel.y += ay * halfDt;
    //     vel.z += az * halfDt;
    //
    //     posX.value += vel.x * dt;
    //     posY.value += vel.y * dt;
    //     posZ.value += vel.z * dt;
    //
    //     force = {0.0, 0.0, 0.0};
    // }
}

void physics::integration::Verlet::postIntegrate(common::WorldState& world, double dt)
{
    const double halfDt = 0.5 * dt;

    // TODO postIntegrate World State
    // for (auto entity : view) {
    //     auto& vel = view.get<physics::components::Velocity>(entity);
    //     const auto& force = view.get<physics::components::ForceAccumulator>(entity);
    //     const auto& mass = view.get<physics::components::ScalarMass>(entity);
    //
    //     if (mass.value == 0.0)
    //         continue;
    //
    //     double ax = force.x / mass.value;
    //     double ay = force.y / mass.value;
    //     double az = force.z / mass.value;
    //
    //     vel.x += ax * halfDt;
    //     vel.y += ay * halfDt;
    //     vel.z += az * halfDt;
    // }
}
