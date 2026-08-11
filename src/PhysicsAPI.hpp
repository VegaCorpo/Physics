#pragma once

#include <memory>
#include <string>
#include <interfaces/IPhysicsEngine.hpp>

extern "C" {
    std::unique_ptr<Physics::IPhysicsEngine> get_engine();
}
