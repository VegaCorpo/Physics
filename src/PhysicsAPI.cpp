#include "NewtonianPhysics.hpp"

static physics::NewtonianPhysics solver;

extern "C++" {
    void physics_init(entt::registry& registry)
    {
        solver.init(registry);
    }

    void physics_update(entt::registry& registry, double dt)
    {
        solver.update(registry, dt);
    }

    void physics_shutdown(entt::registry& registry)
    {
        solver.shutdown(registry);
    }
}
