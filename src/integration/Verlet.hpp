#pragma once

#include <string>

namespace physics {
    namespace integration {
        class Verlet {
            public:
                static std::string get_type() { return "Verlet"; };
        };
    } // namespace integration
} // namespace physics