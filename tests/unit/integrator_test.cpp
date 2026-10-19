#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>
#include <types/types.hpp>
#include <types/World.hpp>
#include "Fixture.hpp"
#include "integration/IntegratorCatalog.hpp"
#include "integration/Pefrl.hpp"
#include "integration/Verlet.hpp"

namespace {

    using physics::NewtonianState;
    using physics::integration::Bodies;
    using physics::integration::IAccelerationField;
    using physics::integration::IIntegrator;
    using physics::integration::IntegratorCatalog;
    using physics::integration::IntegratorError;
    using physics::integration::Pefrl;
    using physics::integration::Verlet;

    constexpr double ANGULAR_FREQUENCY = 1.0;
    constexpr double AMPLITUDE = 1.0;
    constexpr double PERIOD = 2.0 * std::numbers::pi / ANGULAR_FREQUENCY;
    constexpr std::size_t COARSE_STEPS_PER_PERIOD = 32;
    constexpr std::size_t REFINEMENT = 2;
    constexpr double ORDER_TOLERANCE = 0.1;
    constexpr double VERLET_ORDER = 2.0;
    constexpr double PEFRL_ORDER = 4.0;
    constexpr std::size_t VERLET_EVALUATIONS_PER_STEP = 1;
    constexpr std::size_t PEFRL_EVALUATIONS_PER_STEP = 4;
    constexpr std::size_t STALE_REFRESH = 1;
    constexpr double STEP = 0.1;
    constexpr std::size_t REVERSIBILITY_STEPS = 100;
    constexpr double REVERSIBILITY_TOLERANCE = 1e-12;
    constexpr int UNKNOWN_INTEGRATOR_VALUE = 99;

    class SpringField : public IAccelerationField {
        public:
            void compute(NewtonianState& state) override
            {
                this->_evaluations += 1;
                state.forceX[0] = -ANGULAR_FREQUENCY * ANGULAR_FREQUENCY * state.posX[0];
                state.forceY[0] = 0.0;
                state.forceZ[0] = 0.0;
            }

            [[nodiscard]] std::size_t evaluations() const { return this->_evaluations; }

        private:
            std::size_t _evaluations = 0;
    };

    class Oscillator {
        public:
            Oscillator() { this->_state.syncIn(this->_world); }

            [[nodiscard]] Bodies bodies() { return {this->_world, this->_state}; }

            [[nodiscard]] const NewtonianState& state() const { return this->_state; }

            [[nodiscard]] double distanceTo(double position, double velocity) const
            {
                return std::hypot(this->_state.posX[0] - position,
                                  (this->_world.velocities[0].x - velocity) / ANGULAR_FREQUENCY);
            }

        private:
            common::SpecificDataPhysics _world = unit::worldOf({{AMPLITUDE, 0.0, 0.0, 0.0}});
            NewtonianState _state;
    };

    void advance(const IIntegrator& integrator, Oscillator& oscillator, double step, std::size_t steps)
    {
        SpringField field;

        for (std::size_t index = 0; index < steps; index += 1)
            integrator.step(oscillator.bodies(), field, step);
    }

    double errorAfterOnePeriod(const IIntegrator& integrator, std::size_t steps)
    {
        Oscillator oscillator;

        advance(integrator, oscillator, PERIOD / static_cast<double>(steps), steps);
        return oscillator.distanceTo(AMPLITUDE, 0.0);
    }

    double observedOrder(const IIntegrator& integrator)
    {
        const double coarse = errorAfterOnePeriod(integrator, COARSE_STEPS_PER_PERIOD);
        const double fine = errorAfterOnePeriod(integrator, COARSE_STEPS_PER_PERIOD * REFINEMENT);

        return std::log(coarse / fine) / std::log(static_cast<double>(REFINEMENT));
    }

    std::size_t evaluationsForOneStep(const IIntegrator& integrator, Oscillator& oscillator)
    {
        SpringField field;

        integrator.step(oscillator.bodies(), field, STEP);
        return field.evaluations();
    }

    double returnDistance(const IIntegrator& integrator)
    {
        Oscillator oscillator;

        advance(integrator, oscillator, STEP, REVERSIBILITY_STEPS);
        advance(integrator, oscillator, -STEP, REVERSIBILITY_STEPS);
        return oscillator.distanceTo(AMPLITUDE, 0.0);
    }

} // namespace

TEST(Integrator, VerletIsSecondOrder)
{
    EXPECT_NEAR(observedOrder(Verlet{}), VERLET_ORDER, ORDER_TOLERANCE);
}

TEST(Integrator, PefrlIsFourthOrder)
{
    EXPECT_NEAR(observedOrder(Pefrl{}), PEFRL_ORDER, ORDER_TOLERANCE);
}

TEST(Integrator, PefrlIsMoreAccurateThanVerletAtEqualCost)
{
    const double verletError = errorAfterOnePeriod(Verlet{}, COARSE_STEPS_PER_PERIOD * PEFRL_EVALUATIONS_PER_STEP);

    EXPECT_LT(errorAfterOnePeriod(Pefrl{}, COARSE_STEPS_PER_PERIOD), verletError);
}

TEST(Integrator, VerletRefreshesStaleForcesThenReusesThem)
{
    Oscillator oscillator;

    ASSERT_TRUE(oscillator.state().forcesStale());
    EXPECT_EQ(evaluationsForOneStep(Verlet{}, oscillator), STALE_REFRESH + VERLET_EVALUATIONS_PER_STEP);
    EXPECT_FALSE(oscillator.state().forcesStale());
    EXPECT_EQ(evaluationsForOneStep(Verlet{}, oscillator), VERLET_EVALUATIONS_PER_STEP);
}

TEST(Integrator, PefrlEvaluatesForcesFourTimesAndLeavesThemStale)
{
    Oscillator oscillator;

    EXPECT_EQ(evaluationsForOneStep(Pefrl{}, oscillator), PEFRL_EVALUATIONS_PER_STEP);
    EXPECT_TRUE(oscillator.state().forcesStale());
    EXPECT_EQ(evaluationsForOneStep(Pefrl{}, oscillator), PEFRL_EVALUATIONS_PER_STEP);
}

TEST(Integrator, SwitchingFromPefrlToVerletRefreshesTheForces)
{
    Oscillator oscillator;

    advance(Pefrl{}, oscillator, STEP, REVERSIBILITY_STEPS);
    EXPECT_EQ(evaluationsForOneStep(Verlet{}, oscillator), STALE_REFRESH + VERLET_EVALUATIONS_PER_STEP);
}

TEST(Integrator, BothIntegratorsAreTimeReversible)
{
    EXPECT_LT(returnDistance(Verlet{}), REVERSIBILITY_TOLERANCE);
    EXPECT_LT(returnDistance(Pefrl{}), REVERSIBILITY_TOLERANCE);
}

TEST(IntegratorCatalog, EachTypeMapsToTheMatchingIntegrator)
{
    const IntegratorCatalog catalog;
    const auto verlet = catalog.find(common::IntegratorType::VERLET);
    const auto pefrl = catalog.find(common::IntegratorType::PEFRL);

    ASSERT_TRUE(verlet.has_value());
    ASSERT_TRUE(pefrl.has_value());
    EXPECT_NEAR(observedOrder(verlet->get()), VERLET_ORDER, ORDER_TOLERANCE);
    EXPECT_NEAR(observedOrder(pefrl->get()), PEFRL_ORDER, ORDER_TOLERANCE);
}

TEST(IntegratorCatalog, DefaultIntegratorIsVerlet)
{
    EXPECT_EQ(common::DEFAULT_INTEGRATOR, common::IntegratorType::VERLET);
    EXPECT_EQ(common::SpecificDataPhysics{}.integrator, common::IntegratorType::VERLET);
}

TEST(IntegratorCatalog, UnknownTypeIsRejected)
{
    const auto unknown = IntegratorCatalog{}.find(static_cast<common::IntegratorType>(UNKNOWN_INTEGRATOR_VALUE));

    ASSERT_FALSE(unknown.has_value());
    EXPECT_EQ(unknown.error(), IntegratorError::UNKNOWN_INTEGRATOR);
}
