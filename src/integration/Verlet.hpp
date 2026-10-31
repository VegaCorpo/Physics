#pragma once

#include <cstddef>
#include "integration/IIntegrator.hpp"

namespace physics::integration {
    class Verlet : public IIntegrator {
        public:
            void step(const Bodies& bodies, IAccelerationField& field, double duration) const override;

            [[nodiscard]] std::size_t forceEvaluationsPerStep() const noexcept override
            {
                return FORCE_EVALUATIONS_PER_STEP;
            }

        private:
            static constexpr double HALF = 0.5;
            static constexpr std::size_t FORCE_EVALUATIONS_PER_STEP = 1;

            static void _refreshStaleForces(NewtonianState& state, IAccelerationField& field);
    };
} // namespace physics::integration
