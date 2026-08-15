#include "PhysicsAPI.hpp"
#include <entt/signal/fwd.hpp>
#include "NewtonianPhysics.hpp"

extern "C" {
    std::unique_ptr<common::IPhysicsEngine> get_engine()
    {
        return std::make_unique<physics::NewtonianPhysics>();
    }
}
