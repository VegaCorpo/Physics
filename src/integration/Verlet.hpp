#pragma once

#include <entt/entt.hpp>

namespace physics {
    namespace integration {
        class Verlet {
            public:
                static void integrate(entt::registry& registry, double dt);
            private:
                static void _computeAcceleration(entt::registry& registry);
                static void _updatePositionAndVelocity(entt::registry& registry, double dt);
        };
    } // namespace integration
} // namespace physics