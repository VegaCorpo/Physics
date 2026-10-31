#include "PhysicsAPI.hpp"
#include <entt/signal/fwd.hpp>
#include "NewtonianPhysics.hpp"
#include <types/ModuleInfo.hpp>

extern "C" {
    std::unique_ptr<common::IPhysicsEngine> get_engine()
    {
        return std::make_unique<physics::NewtonianPhysics>();
    }

    common::ModuleInfo get_module_info()
    {
        return {
            .name = ORBITAL_MODULE_NAME,
            .version = ORBITAL_MODULE_VERSION,
            .type = common::ModuleType::PHYSICS,
        };
    }
}
