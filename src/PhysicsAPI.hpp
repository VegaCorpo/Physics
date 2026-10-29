#pragma once

#include <interfaces/IPhysicsEngine.hpp>
#include <memory>
#include <string>
#include <types/ModuleInfo.hpp>

extern "C" {
    std::unique_ptr<common::IPhysicsEngine> get_engine();
}
