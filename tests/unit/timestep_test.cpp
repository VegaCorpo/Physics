#include "integration/TimeStep.hpp"
#include <array>
#include <cstddef>
#include <expected>
#include <gtest/gtest.h>
#include <limits>

namespace {

    using physics::integration::SubSteps;
    using physics::integration::TimeStep;
    using physics::integration::TimeStepError;

    constexpr double MAX_SUB_STEP = TimeStep::DEFAULT_MAX_SUB_STEP;
    constexpr double SECONDS_PER_DAY = 86400.0;
    constexpr double SECONDS_PER_JULIAN_YEAR = 31557600.0;
    constexpr double ONE_MINUTE = 60.0;
    constexpr double CUSTOM_MAX_SUB_STEP = 0.25;
    constexpr std::size_t CUSTOM_SUB_STEPS_PER_SECOND = 4;
    constexpr std::size_t SUB_STEPS_PER_DAY = 12;
    constexpr std::size_t SUB_STEPS_PER_JULIAN_YEAR = 4383;
    constexpr std::size_t HALVES = 2;
    constexpr double ONE_SECOND = 1.0;
    constexpr double RELATIVE_ROUNDING = 4.0 * std::numeric_limits<double>::epsilon();
    constexpr std::array DEFAULT_FRAME_SAMPLES = {0.001,   1.0,     7199.999,  7200.001,
                                                  14400.0, 99999.9, 2629800.0, 315576000.0};
    constexpr std::array CUSTOM_FRAME_SAMPLES = {0.1, 0.3, 1.0, 3.7, 1000.0};

    SubSteps splitOrFail(const TimeStep& timeStep, double frameDuration)
    {
        const std::expected<SubSteps, TimeStepError> subSteps = timeStep.split(frameDuration);

        EXPECT_TRUE(subSteps.has_value()) << "frame duration " << frameDuration << " was rejected";
        return subSteps.value_or(SubSteps{});
    }

    TimeStepError errorOf(double frameDuration)
    {
        const std::expected<SubSteps, TimeStepError> subSteps = TimeStep{}.split(frameDuration);

        EXPECT_FALSE(subSteps.has_value()) << "frame duration " << frameDuration << " was accepted";
        return subSteps.error_or(TimeStepError::NOT_FINITE);
    }

    void expectCoversFrame(const TimeStep& timeStep, double frameDuration, double maxSubStep)
    {
        const SubSteps subSteps = splitOrFail(timeStep, frameDuration);
        const double covered = subSteps.duration * static_cast<double>(subSteps.count);
        const double coveredWithOneLess = maxSubStep * static_cast<double>(subSteps.count - 1);

        EXPECT_LE(subSteps.duration, maxSubStep * (1.0 + RELATIVE_ROUNDING)) << "frame duration " << frameDuration;
        EXPECT_NEAR(covered, frameDuration, frameDuration * RELATIVE_ROUNDING) << "frame duration " << frameDuration;
        EXPECT_LT(coveredWithOneLess, frameDuration) << "more sub-steps than needed for " << frameDuration;
    }

} // namespace

TEST(TimeStep, FrameEqualToTheMaxSubStepIsASingleIdenticalStep)
{
    const SubSteps subSteps = splitOrFail(TimeStep{}, MAX_SUB_STEP);

    EXPECT_EQ(subSteps.count, 1U);
    EXPECT_EQ(subSteps.duration, MAX_SUB_STEP);
}

TEST(TimeStep, FrameShorterThanTheMaxSubStepIsKeptAsIs)
{
    const SubSteps subSteps = splitOrFail(TimeStep{}, ONE_MINUTE);

    EXPECT_EQ(subSteps.count, 1U);
    EXPECT_EQ(subSteps.duration, ONE_MINUTE);
}

TEST(TimeStep, ZeroFrameDurationNeedsNoSubStep)
{
    EXPECT_TRUE(splitOrFail(TimeStep{}, 0.0).empty());
    EXPECT_TRUE(splitOrFail(TimeStep{}, -0.0).empty());
}

TEST(TimeStep, ExactMultipleIsSplitIntoMaxSubSteps)
{
    const SubSteps subSteps = splitOrFail(TimeStep{}, SECONDS_PER_DAY);

    EXPECT_EQ(subSteps.count, SUB_STEPS_PER_DAY);
    EXPECT_EQ(subSteps.duration, MAX_SUB_STEP);
}

TEST(TimeStep, UserScaleOfOneYearIsSplitIntoStableSubSteps)
{
    const SubSteps subSteps = splitOrFail(TimeStep{}, SECONDS_PER_JULIAN_YEAR);

    EXPECT_EQ(subSteps.count, SUB_STEPS_PER_JULIAN_YEAR);
    EXPECT_EQ(subSteps.duration, MAX_SUB_STEP);
}

TEST(TimeStep, NonMultipleIsSplitIntoEqualSubStepsBelowTheMax)
{
    const double justAboveMax = MAX_SUB_STEP + ONE_SECOND;
    const SubSteps subSteps = splitOrFail(TimeStep{}, justAboveMax);

    EXPECT_EQ(subSteps.count, HALVES);
    EXPECT_DOUBLE_EQ(subSteps.duration, justAboveMax / static_cast<double>(HALVES));
}

TEST(TimeStep, SubStepsAlwaysCoverTheWholeFrameWithoutExceedingTheMax)
{
    const TimeStep timeStep;

    for (const double frameDuration : DEFAULT_FRAME_SAMPLES)
        expectCoversFrame(timeStep, frameDuration, MAX_SUB_STEP);
}

TEST(TimeStep, CustomMaxSubStepIsHonoured)
{
    const std::expected<TimeStep, TimeStepError> timeStep = TimeStep::withMaxSubStep(CUSTOM_MAX_SUB_STEP);

    ASSERT_TRUE(timeStep.has_value());
    EXPECT_EQ(splitOrFail(*timeStep, ONE_SECOND).count, CUSTOM_SUB_STEPS_PER_SECOND);
    for (const double frameDuration : CUSTOM_FRAME_SAMPLES)
        expectCoversFrame(*timeStep, frameDuration, CUSTOM_MAX_SUB_STEP);
}

TEST(TimeStep, NegativeFrameDurationIsRejected)
{
    EXPECT_EQ(errorOf(-MAX_SUB_STEP), TimeStepError::NEGATIVE);
    EXPECT_EQ(errorOf(-std::numeric_limits<double>::denorm_min()), TimeStepError::NEGATIVE);
}

TEST(TimeStep, NonFiniteFrameDurationIsRejected)
{
    EXPECT_EQ(errorOf(std::numeric_limits<double>::quiet_NaN()), TimeStepError::NOT_FINITE);
    EXPECT_EQ(errorOf(std::numeric_limits<double>::infinity()), TimeStepError::NOT_FINITE);
    EXPECT_EQ(errorOf(-std::numeric_limits<double>::infinity()), TimeStepError::NOT_FINITE);
}

TEST(TimeStep, FrameNeedingTooManySubStepsIsRejected)
{
    const double justTooLong = MAX_SUB_STEP * static_cast<double>(TimeStep::MAX_SUB_STEP_COUNT + 1);

    EXPECT_EQ(errorOf(justTooLong), TimeStepError::TOO_MANY_SUB_STEPS);
    EXPECT_EQ(errorOf(std::numeric_limits<double>::max()), TimeStepError::TOO_MANY_SUB_STEPS);
}

TEST(TimeStep, LongestAcceptedFrameUsesTheMaximumSubStepCount)
{
    const double longest = MAX_SUB_STEP * static_cast<double>(TimeStep::MAX_SUB_STEP_COUNT);

    EXPECT_EQ(splitOrFail(TimeStep{}, longest).count, TimeStep::MAX_SUB_STEP_COUNT);
}

TEST(TimeStep, InvalidMaxSubStepIsRejected)
{
    EXPECT_EQ(TimeStep::withMaxSubStep(0.0).error(), TimeStepError::ZERO_SUB_STEP);
    EXPECT_EQ(TimeStep::withMaxSubStep(-ONE_SECOND).error(), TimeStepError::NEGATIVE);
    EXPECT_EQ(TimeStep::withMaxSubStep(std::numeric_limits<double>::quiet_NaN()).error(), TimeStepError::NOT_FINITE);
    EXPECT_EQ(TimeStep::withMaxSubStep(std::numeric_limits<double>::infinity()).error(), TimeStepError::NOT_FINITE);
}

TEST(TimeStep, TinyMaxSubStepCannotOverflowTheSubStepCount)
{
    const std::expected<TimeStep, TimeStepError> timeStep =
        TimeStep::withMaxSubStep(std::numeric_limits<double>::denorm_min());

    ASSERT_TRUE(timeStep.has_value());
    EXPECT_EQ(timeStep->split(ONE_SECOND).error(), TimeStepError::TOO_MANY_SUB_STEPS);
}
