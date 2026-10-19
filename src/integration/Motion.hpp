#pragma once

#include "integration/IIntegrator.hpp"

namespace physics::integration::motion {
    void kick(const Bodies& bodies, double duration);
    void drift(const Bodies& bodies, double duration);
} // namespace physics::integration::motion
