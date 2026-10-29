#include "PhysicsAPI.hpp"
#include <entt/signal/fwd.hpp>
#include "NewtonianPhysics.hpp"

extern "C" {
    std::unique_ptr<common::IPhysicsEngine> get_engine()
    {
        return std::make_unique<physics::NewtonianPhysics>();
    }

    const common::ModuleInfo* get_module_info()
    {
        static constexpr common::ModuleInfo info{
            .name = ORBITAL_MODULE_NAME,
            .version = ORBITAL_MODULE_VERSION,
            .type = common::ModuleType::PHYSICS,
        };
        return &info;
    }
}
