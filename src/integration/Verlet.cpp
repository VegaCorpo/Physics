#include "Verlet.hpp"
#include "components/NewtonianState.hpp"

void physics::integration::Verlet::preIntegrate(common::WorldState& world, NewtonianState& state, double dt)
{
    const double halfDt = 0.5 * dt;

    auto& vel = world.velocities;

    auto& posX = state.posX;
    auto& posY = state.posY;
    auto& posZ = state.posZ;
    auto& forceX = state.forceX;
    auto& forceY = state.forceY;
    auto& forceZ = state.forceZ;

    for (size_t i = 0; i < state.size(); i += 1) {
        vel[i].x += forceX[i] * halfDt;
        vel[i].y += forceY[i] * halfDt;
        vel[i].z += forceZ[i] * halfDt;

        posX[i] += vel[i].x * dt;
        posY[i] += vel[i].y * dt;
        posZ[i] += vel[i].z * dt;

        forceX[i] = 0;
        forceY[i] = 0;
        forceZ[i] = 0;
    }
}

void physics::integration::Verlet::postIntegrate(common::WorldState& world, NewtonianState& state, double dt)
{
    const double halfDt = 0.5 * dt;

    auto& vel = world.velocities;
    const auto& forceX = state.forceX;
    const auto& forceY = state.forceY;
    const auto& forceZ = state.forceZ;

    for (size_t i = 0; i < state.size(); i += 1) {
        vel[i].x += forceX[i] * halfDt;
        vel[i].y += forceY[i] * halfDt;
        vel[i].z += forceZ[i] * halfDt;
    }
}
