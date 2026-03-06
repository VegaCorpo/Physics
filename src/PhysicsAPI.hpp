#pragma once

#include <string>

extern "C" {
    std::string getName();
    void physicsInit(void* registry_ptr, void* dispatcher_ptr);
    void physicsUpdate(void* registry_ptr, void* dispatcher_ptr, double dt);
    void physicsShutdown(void* registry_ptr);
    void physicsSyncIn(void* registry_ptr);
    void physicsSyncOut(void* registry_ptr);
}
