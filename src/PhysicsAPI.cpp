#include "PhysicsAPI.hpp"
#include <cstring>
#include <entt/signal/fwd.hpp>
#include <iostream>
#include "entt/entity/registry.hpp"
#include "NewtonianPhysics.hpp"

extern "C" {
    std::unique_ptr<common::IPhysicsEngine> get_engine() {
    return std::make_unique<physics::NewtonianPhysics>();
}
}
