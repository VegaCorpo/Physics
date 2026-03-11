#include "PhysicsAPI.hpp"
#include <cstring>
#include <entt/signal/fwd.hpp>
#include <iostream>
#include "entt/entity/registry.hpp"
#include "NewtonianPhysics.hpp"

extern "C" {
    std::string getName()
    {
        return physics::NewtonianPhysics::getName();
    }

    void physicsInit(void* registry_ptr, void* dispatcher_ptr)
    {
        auto& registry = *static_cast<entt::registry*>(registry_ptr);
        auto& dispatcher = *static_cast<entt::dispatcher*>(dispatcher_ptr);
        physics::NewtonianPhysics::init(registry, dispatcher);
    }

    void physicsUpdate(void* registry_ptr, void* dispatcher_ptr, double dt)
    {
        auto& registry = *static_cast<entt::registry*>(registry_ptr);
        auto& dispatcher = *static_cast<entt::dispatcher*>(dispatcher_ptr);
        physics::NewtonianPhysics::update(registry, dispatcher, dt);
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
