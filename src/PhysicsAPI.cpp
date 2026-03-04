#include "PhysicsAPI.hpp"
#include "NewtonianPhysics.hpp"

extern "C++" {
    std::string getName()
    {
        return physics::NewtonianPhysics::getName();
    }

    void physicsInit(entt::registry& registry)
    {
        physics::NewtonianPhysics::init(registry);
    }

    void physicsUpdate(entt::registry& registry, double dt)
    {
        physics::NewtonianPhysics::update(registry, dt);
    }

    void physicsShutdown(entt::registry& registry)
    {
        physics::NewtonianPhysics::shutdown(registry);
    }
}
