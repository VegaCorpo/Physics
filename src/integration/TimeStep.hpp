#pragma once

#include <cstddef>
#include <expected>

namespace physics::integration {
    enum class TimeStepError {
        NOT_FINITE,
        NEGATIVE,
        ZERO_SUB_STEP,
        TOO_MANY_SUB_STEPS,
    };

    struct SubSteps {
            std::size_t count = 0;
            double duration = 0.0;

            [[nodiscard]] bool empty() const noexcept { return count == 0; }
    };

    class TimeStep {
        public:
            static constexpr double DEFAULT_MAX_SUB_STEP = 7200.0;
            static constexpr std::size_t MAX_SUB_STEP_COUNT = 10'000'000;

            TimeStep() = default;

            [[nodiscard]] static std::expected<TimeStep, TimeStepError> withMaxSubStep(double maxSubStep);

            [[nodiscard]] std::expected<SubSteps, TimeStepError> split(double frameDuration) const;

        private:
            explicit TimeStep(double maxSubStep) : _maxSubStep(maxSubStep) {}

            [[nodiscard]] static std::expected<double, TimeStepError> _validate(double duration);
            [[nodiscard]] std::expected<SubSteps, TimeStepError> _divide(double duration) const;

            double _maxSubStep = DEFAULT_MAX_SUB_STEP;
    };
} // namespace physics::integration
