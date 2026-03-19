#pragma once

#include <entt/entt.hpp>

namespace physics::integration {
    class Verlet {
        public:
            static void preIntegrate(entt::registry& registry, double dt);
            static void postIntegrate(entt::registry& registry, double dt);

        private:
    };
} // namespace physics::integration
