#pragma once

namespace physics {
    namespace components {

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
    } // namespace components
} // namespace physics
