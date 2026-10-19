#pragma once

#include "integration/IIntegrator.hpp"

namespace physics::integration {
    class Verlet : public IIntegrator {
        public:
            void step(const Bodies& bodies, IAccelerationField& field, double duration) const override;

        private:
            static constexpr double HALF = 0.5;

            static void _refreshStaleForces(NewtonianState& state, IAccelerationField& field);
    };
} // namespace physics::integration
