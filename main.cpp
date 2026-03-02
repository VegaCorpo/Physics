#include <iostream>
#include <memory>
#include <string>

namespace physics {
    class IPhysicsEngine {};
    class NewtonianEngine : public IPhysicsEngine {
        public:
            static std::string get_type() { return "Newtonian"; };
    };
} // namespace physics

extern "C++" {
std::unique_ptr<physics::NewtonianEngine> get_physics_engine()
{
    std::cout << "Hello from physics engine" << std::endl;
    return std::make_unique<physics::NewtonianEngine>();
}
}
