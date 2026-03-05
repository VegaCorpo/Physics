#pragma once

#include <entt/entt.hpp>

extern "C++" {
    std::string getName();
    void physicsInit(entt::registry& registry);
    void physicsUpdate(entt::registry& registry, double dt);
    void physicsShutdown(entt::registry& registry);
    void physicsSyncIn(entt::registry& registry);
    void physicsSyncOut(entt::registry& registry);
}
