#pragma once

#include <string>

namespace physics {
    namespace integration {
        class Verlet {
            public:
                static std::string getType() { return "Verlet"; };
        };
    } // namespace integration
} // namespace physics