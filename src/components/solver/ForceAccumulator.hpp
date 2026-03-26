#pragma once

namespace physics::components {
    struct alignas(16) ForceAccumulator {
            double x;
            double y;
            double z;
    };
} // namespace physics::components
