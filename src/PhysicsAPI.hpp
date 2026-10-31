#pragma once

#include <interfaces/IPhysicsEngine.hpp>
#include <memory>

extern "C" {
    std::unique_ptr<common::IPhysicsEngine> get_engine();
}
