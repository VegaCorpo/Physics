#pragma once

#include <entt/entt.hpp>

extern "C++" {
    void physicsInit(entt::registry& registry);
    void physicsUpdate(entt::registry& registry, double dt);
    void physicsShutdown(entt::registry& registry);
}
