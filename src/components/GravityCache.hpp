#pragma once

namespace physics::components {

    struct ScalarMass {
            double value; // mantissa * 10^exponent
    };

    struct Displacement {
            double dx;
            double dy;
            double dz;
    };

    struct InverseDistance {
            double invDistCubed;
    };
} // namespace physics::components
