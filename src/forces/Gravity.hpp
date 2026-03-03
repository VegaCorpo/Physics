#pragma once

#include <string>

namespace physics {
    namespace forces {
        class Gravity {
            public:
                static std::string get_type() { return "Gravity"; };
        };
    } // namespace forces
} // namespace physics
