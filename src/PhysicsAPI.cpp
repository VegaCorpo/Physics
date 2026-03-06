#include "PhysicsAPI.hpp"
#include <cstring>
#include <iostream>
#include "entt/entity/registry.hpp"
#include "NewtonianPhysics.hpp"

extern "C" {
    std::string getName()
    {
        return physics::NewtonianPhysics::getName();
    }

    void physicsInit(void* registry_ptr)
    {
        auto& registry = *static_cast<entt::registry*>(registry_ptr);
        physics::NewtonianPhysics::init(registry);
    }

    void physicsUpdate(void* registry_ptr, double dt)
    {
        auto& registry = *static_cast<entt::registry*>(registry_ptr);
        std::cout << "Hello from physics update" << std::endl;
        physics::NewtonianPhysics::update(registry, dt);
    }

    void physicsShutdown(void* registry_ptr)
    {
        auto& registry = *static_cast<entt::registry*>(registry_ptr);
        physics::NewtonianPhysics::shutdown(registry);
    }

    void physicsSyncIn(void* registry_ptr)
    {
        auto& registry = *static_cast<entt::registry*>(registry_ptr);
        physics::NewtonianPhysics::syncIn(registry);
    }

    void physicsSyncOut(void* registry_ptr)
    {
        auto& registry = *static_cast<entt::registry*>(registry_ptr);
        physics::NewtonianPhysics::syncOut(registry);
    }
}
