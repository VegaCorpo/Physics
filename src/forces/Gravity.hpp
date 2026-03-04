#pragma once

#include <string>

namespace physics {
    namespace forces {
        class Gravity {
            public:
                static std::string getType() { return "Gravity"; };
        };
    } // namespace forces
} // namespace physics
