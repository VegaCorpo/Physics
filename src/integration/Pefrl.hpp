#pragma once

#include "integration/IIntegrator.hpp"

namespace physics::integration {
    class Pefrl : public IIntegrator {
        public:
            void step(const Bodies& bodies, IAccelerationField& field, double duration) const override;

        private:
            static constexpr double XI = 0.1786178958448091;
            static constexpr double LAMBDA = -0.2123418310626054;
            static constexpr double CHI = -0.06626458266981849;
            static constexpr double OUTER_KICK = (1.0 - 2.0 * LAMBDA) / 2.0;
            static constexpr double MIDDLE_DRIFT = 1.0 - 2.0 * (CHI + XI);

            static void _kick(const Bodies& bodies, IAccelerationField& field, double duration);
    };
} // namespace physics::integration
