#include "Verlet.hpp"
#include "components/NewtonianState.hpp"

void physics::integration::Verlet::preIntegrate(common::SpecificDataPhysics& world, NewtonianState& state, double dt)
{
    const double halfDt = 0.5 * dt;

    auto& vel = world.velocities;

    auto& posX = state.posX;
    auto& posY = state.posY;
    auto& posZ = state.posZ;
    auto& forceX = state.forceX;
    auto& forceY = state.forceY;
    auto& forceZ = state.forceZ;

    auto& mass = state.scalarMass;

    for (size_t i = 0; i < state.size(); i += 1) {
        if (mass[i] == 0.0)
            continue;

        double ax = forceX[i] / mass[i];
        double ay = forceY[i] / mass[i];
        double az = forceZ[i] / mass[i];

        vel[i].x += ax * halfDt;
        vel[i].y += ay * halfDt;
        vel[i].z += az * halfDt;

        posX[i] += vel[i].x * dt;
        posY[i] += vel[i].y * dt;
        posZ[i] += vel[i].z * dt;

        forceX[i] = 0;
        forceY[i] = 0;
        forceZ[i] = 0;
    }
}

void physics::integration::Verlet::postIntegrate(common::SpecificDataPhysics& world, NewtonianState& state, double dt)
{
    const double halfDt = 0.5 * dt;

    auto& vel = world.velocities;
    const auto& forceX = state.forceX;
    const auto& forceY = state.forceY;
    const auto& forceZ = state.forceZ;
    const auto& mass = state.scalarMass;

    for (size_t i = 0; i < state.size(); i += 1) {
        if (mass[i] == 0.0)
            continue;

        double ax = forceX[i] / mass[i];
        double ay = forceY[i] / mass[i];
        double az = forceZ[i] / mass[i];

        vel[i].x += ax * halfDt;
        vel[i].y += ay * halfDt;
        vel[i].z += az * halfDt;
    }
}
