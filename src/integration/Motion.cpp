#include "Motion.hpp"
#include <cstddef>

void physics::integration::motion::kick(const Bodies& bodies, double duration)
{
    auto& velocities = bodies.world.velocities;
    const NewtonianState& state = bodies.state;

    for (std::size_t body = 0; body < state.size(); body += 1) {
        velocities[body].x += state.forceX[body] * duration;
        velocities[body].y += state.forceY[body] * duration;
        velocities[body].z += state.forceZ[body] * duration;
    }
}

void physics::integration::motion::drift(const Bodies& bodies, double duration)
{
    const auto& velocities = bodies.world.velocities;
    NewtonianState& state = bodies.state;

    for (std::size_t body = 0; body < state.size(); body += 1) {
        state.posX[body] += velocities[body].x * duration;
        state.posY[body] += velocities[body].y * duration;
        state.posZ[body] += velocities[body].z * duration;
    }
}
