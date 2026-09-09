#pragma once

#include <limits>

namespace physics {
    struct Min {
            double X = std::numeric_limits<double>::infinity();
            double Y = std::numeric_limits<double>::infinity();
            double Z = std::numeric_limits<double>::infinity();
    }; // Size 24

    struct Max {
            double X = std::numeric_limits<double>::lowest();
            double Y = std::numeric_limits<double>::lowest();
            double Z = std::numeric_limits<double>::lowest();
    }; // Size 24

    struct Bounds {
            Min posMin;
            Max posMax;
    }; // Size 48

}; // namespace physics
