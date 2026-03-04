#include "PhysicsAPI.hpp"
#include "NewtonianPhysics.hpp"

static physics::NewtonianPhysics solver;

extern "C++" {
    void physicsInit(entt::registry& registry)
    {
        solver.init(registry);
    }

    void physicsUpdate(entt::registry& registry, double dt)
    {
        solver.update(registry, dt);
    }

    void physicsShutdown(entt::registry& registry)
    {
        solver.shutdown(registry);
    }
}
