#pragma once

namespace physics::components {
    struct alignas(16) Mass {
            float mantissa;
            int exponent;
    };
} // namespace physics::components
