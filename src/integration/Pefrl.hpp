#pragma once

#include <cstddef>
#include "integration/IIntegrator.hpp"

namespace physics::integration {
    class Pefrl : public IIntegrator {
        public:
            void step(const Bodies& bodies, IAccelerationField& field, double duration) const override;

            [[nodiscard]] std::size_t forceEvaluationsPerStep() const noexcept override
            {
                return FORCE_EVALUATIONS_PER_STEP;
            }

        private:
            static constexpr double XI = 0.1786178958448091;
            static constexpr double LAMBDA = -0.2123418310626054;
            static constexpr double CHI = -0.06626458266981849;
            static constexpr double OUTER_KICK = (1.0 - 2.0 * LAMBDA) / 2.0;
            static constexpr double MIDDLE_DRIFT = 1.0 - 2.0 * (CHI + XI);
            static constexpr std::size_t FORCE_EVALUATIONS_PER_STEP = 4;

            static void _kick(const Bodies& bodies, IAccelerationField& field, double duration);
    };
} // namespace physics::integration
