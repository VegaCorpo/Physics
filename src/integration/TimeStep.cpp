#include "TimeStep.hpp"
#include <cmath>

auto physics::integration::TimeStep::withMaxSubStep(double maxSubStep) -> std::expected<TimeStep, TimeStepError>
{
    const std::expected<double, TimeStepError> validated = _validate(maxSubStep);

    if (!validated)
        return std::unexpected(validated.error());
    if (*validated == 0.0)
        return std::unexpected(TimeStepError::ZERO_SUB_STEP);
    return TimeStep(*validated);
}

auto physics::integration::TimeStep::forForceEvaluations(std::size_t evaluationsPerStep)
    -> std::expected<TimeStep, TimeStepError>
{
    return withMaxSubStep(DEFAULT_MAX_SUB_STEP * static_cast<double>(evaluationsPerStep));
}

auto physics::integration::TimeStep::split(double frameDuration) const -> std::expected<SubSteps, TimeStepError>
{
    return _validate(frameDuration).and_then([this](double duration) { return this->_divide(duration); });
}

auto physics::integration::TimeStep::_validate(double duration) -> std::expected<double, TimeStepError>
{
    if (!std::isfinite(duration))
        return std::unexpected(TimeStepError::NOT_FINITE);
    if (duration < 0.0)
        return std::unexpected(TimeStepError::NEGATIVE);
    return duration;
}

auto physics::integration::TimeStep::_divide(double duration) const -> std::expected<SubSteps, TimeStepError>
{
    const double count = std::ceil(duration / this->_maxSubStep);

    if (count > static_cast<double>(MAX_SUB_STEP_COUNT))
        return std::unexpected(TimeStepError::TOO_MANY_SUB_STEPS);
    if (count == 0.0)
        return SubSteps{};
    return SubSteps{static_cast<std::size_t>(count), duration / count};
}
