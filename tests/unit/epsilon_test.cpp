/**
 * White-box tests of the gravitational softening length (epsilon).
 *
 * NewtonianState::syncIn() keeps the epsilon given in SpecificDataPhysics, or
 * derives it when it is 0: SOFTENING_RADIUS_RATIO times the smallest non-zero
 * body radius, DEFAULT_EPSILON when every body is a point mass. Gravity::apply()
 * then uses the Plummer kernel G * m * d / (d^2 + epsilon^2)^(3/2).
 *
 * The Epsilon tests check the value chosen by syncIn(); the Softening tests
 * check that Gravity uses it and that the derived value keeps the force exact
 * outside the bodies while still removing the r -> 0 singularity.
 */

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
#include "Fixture.hpp"
#include "forces/Gravity.hpp"

using unit::Body;

namespace {

    using physics::NewtonianState;
    using physics::forces::G;

    constexpr double MASS = 1.0e24; // Fixture mass: {1.0f, 24}

    common::SpecificDataPhysics worldOf(const std::vector<Body>& bodies, double epsilon)
    {
        common::SpecificDataPhysics world = unit::worldOf(bodies);
        world.epsilon = epsilon;
        return world;
    }

    NewtonianState stateOf(const std::vector<Body>& bodies, double epsilon)
    {
        NewtonianState state;
        state.syncIn(worldOf(bodies, epsilon));
        return state;
    }

    /// Radii go through a float in SpecificDataPhysics: expected values must use the same round trip.
    double derived(double radius)
    {
        return NewtonianState::SOFTENING_RADIUS_RATIO * static_cast<double>(static_cast<float>(radius));
    }

    /// Acceleration of body 0 along +x, pulled by body 1 at distance d on the +x axis.
    double pullAlongX(double distance, double radius, double epsilon)
    {
        NewtonianState state = stateOf({{0.0, 0.0, 0.0, radius}, {distance, 0.0, 0.0, radius}}, epsilon);
        physics::forces::Gravity::apply(state, 0.0);
        return state.forceX[0];
    }

    double plummer(double distance, double epsilon)
    {
        return G * MASS * distance / std::pow(distance * distance + epsilon * epsilon, 1.5);
    }

    double newton(double distance)
    {
        return G * MASS / (distance * distance);
    }

    double relativeError(double actual, double expected)
    {
        return std::abs(actual - expected) / std::abs(expected);
    }

} // namespace

// --------------------------------------------------------------------------- value chosen by syncIn()

TEST(Epsilon, NonZeroEpsilonIsKeptAsIs)
{
    EXPECT_EQ(stateOf({{0.0, 0.0, 0.0, 1000.0}, {1.0e6, 0.0, 0.0, 1000.0}}, 42.0).epsilon, 42.0);
    EXPECT_EQ(stateOf({{0.0, 0.0, 0.0, 1000.0}}, 1.0e-6).epsilon, 1.0e-6);
}

TEST(Epsilon, ZeroEpsilonIsDerivedFromTheSmallestRadius)
{
    // Earth, Moon, Jupiter: the Moon's radius sets the scale.
    const NewtonianState state = stateOf(
        {{0.0, 0.0, 0.0, 6371.0}, {3.844e5, 0.0, 0.0, 1737.4}, {7.785e8, 0.0, 0.0, 69911.0}}, 0.0);
    EXPECT_DOUBLE_EQ(state.epsilon, derived(1737.4));
}

TEST(Epsilon, DerivedEpsilonDoesNotDependOnBodyOrder)
{
    const double expected = derived(250.0);
    EXPECT_DOUBLE_EQ(stateOf({{0, 0, 0, 250.0}, {1e6, 0, 0, 900.0}, {2e6, 0, 0, 4000.0}}, 0.0).epsilon, expected);
    EXPECT_DOUBLE_EQ(stateOf({{0, 0, 0, 4000.0}, {1e6, 0, 0, 900.0}, {2e6, 0, 0, 250.0}}, 0.0).epsilon, expected);
    EXPECT_DOUBLE_EQ(stateOf({{0, 0, 0, 900.0}, {1e6, 0, 0, 250.0}, {2e6, 0, 0, 4000.0}}, 0.0).epsilon, expected);
}

TEST(Epsilon, ZeroRadiiAreIgnoredWhenDerivingEpsilon)
{
    // Point masses mixed with real bodies: the smallest *non-zero* radius is used.
    const NewtonianState state =
        stateOf({{0, 0, 0, 0.0}, {1e6, 0, 0, 500.0}, {2e6, 0, 0, 0.0}, {3e6, 0, 0, 2000.0}}, 0.0);
    EXPECT_DOUBLE_EQ(state.epsilon, derived(500.0));
}

TEST(Epsilon, PointMassesFallBackToTheDefault)
{
    EXPECT_EQ(stateOf({{0, 0, 0, 0.0}, {1e6, 0, 0, 0.0}, {2e6, 0, 0, 0.0}}, 0.0).epsilon,
              NewtonianState::DEFAULT_EPSILON);
}

TEST(Epsilon, EmptyWorldFallsBackToTheDefault)
{
    EXPECT_EQ(stateOf({}, 0.0).epsilon, NewtonianState::DEFAULT_EPSILON);
}

TEST(Epsilon, PaddingSlotsDoNotAffectTheDerivedValue)
{
    // Padding slots have radius 0 and must be skipped, for any count around the SIMD block boundary.
    for (std::size_t n : {1u, 2u, 7u, 8u, 9u, 15u, 16u, 17u, 31u, 32u, 33u}) {
        std::vector<Body> bodies = unit::randomCluster(n, static_cast<unsigned>(n), 1.0e6, 300.0, 900.0);
        double smallest = bodies[0].radius;
        for (const Body& b : bodies)
            smallest = std::min(smallest, b.radius);

        const NewtonianState state = stateOf(bodies, 0.0);
        ASSERT_GE(state.paddedSize(), state.size());
        EXPECT_DOUBLE_EQ(state.epsilon, derived(smallest)) << "n=" << n;
    }
}

TEST(Epsilon, DerivedValueIsRecomputedOnEverySync)
{
    // The engine keeps one NewtonianState: radii changing between frames (e.g. after a merge)
    // or bodies disappearing must update epsilon.
    NewtonianState state;

    state.syncIn(worldOf({{0, 0, 0, 100.0}, {1e6, 0, 0, 200.0}}, 0.0));
    EXPECT_DOUBLE_EQ(state.epsilon, derived(100.0));

    state.syncIn(worldOf({{0, 0, 0, 300.0}, {1e6, 0, 0, 400.0}}, 0.0)); // same count, new radii
    EXPECT_DOUBLE_EQ(state.epsilon, derived(300.0));

    state.syncIn(worldOf({{0, 0, 0, 5000.0}}, 0.0)); // smallest body removed
    EXPECT_DOUBLE_EQ(state.epsilon, derived(5000.0));
}

TEST(Epsilon, SwitchingBetweenExplicitAndDerivedValues)
{
    const std::vector<Body> bodies = {{0, 0, 0, 100.0}, {1e6, 0, 0, 200.0}};
    NewtonianState state;

    state.syncIn(worldOf(bodies, 5.0));
    EXPECT_EQ(state.epsilon, 5.0);

    state.syncIn(worldOf(bodies, 0.0));
    EXPECT_DOUBLE_EQ(state.epsilon, derived(100.0));

    state.syncIn(worldOf(bodies, 7.0));
    EXPECT_EQ(state.epsilon, 7.0);
}

// --------------------------------------------------------------------------- effect on Gravity::apply()

TEST(Softening, ExplicitEpsilonFollowsThePlummerKernel)
{
    // Large epsilon compared with the distance so the softening is clearly visible.
    for (const double epsilon : {1.0, 300.0, 1000.0, 5000.0}) {
        const double d = 1000.0;
        const double actual = pullAlongX(d, 10.0, epsilon);
        EXPECT_LT(relativeError(actual, plummer(d, epsilon)), 1e-12) << "epsilon=" << epsilon;
        if (epsilon >= 300.0)
            EXPECT_LT(actual, newton(d)) << "softening must weaken the force, epsilon=" << epsilon;
    }
}

TEST(Softening, DerivedEpsilonIsUsedByGravity)
{
    const double d = 2.0e3;
    const double r = 1.0e3;
    EXPECT_LT(relativeError(pullAlongX(d, r, 0.0), plummer(d, derived(r))), 1e-12);
}

TEST(Softening, DerivedEpsilonKeepsTheForceExactAtContact)
{
    // Closest physical separation of two bodies is r1 + r2 >= 2 * R_min. There the relative
    // error of the softened force is ~1.5 * (ratio / 2)^2, i.e. ~3.75e-7 for ratio = 1e-3.
    for (const double r : {1.0, 1737.4, 6371.0, 69911.0}) {
        const double d = 2.0 * static_cast<double>(static_cast<float>(r));
        const double err = relativeError(pullAlongX(d, r, 0.0), newton(d));
        EXPECT_LT(err, 1e-6) << "radius=" << r;
        EXPECT_GT(err, 0.0) << "softening must still be applied, radius=" << r;
    }
}

TEST(Softening, DerivedEpsilonIsNegligibleAtOrbitalDistances)
{
    // Earth-Moon distance with the Moon's radius as the smallest body.
    const double d = 3.844e5;
    EXPECT_LT(relativeError(pullAlongX(d, 1737.4, 0.0), newton(d)), 1e-10);
}

TEST(Softening, CoincidentPointMassesProduceFiniteForces)
{
    // Without softening r^2 = 0 gives 0 * inf = NaN. The default epsilon must prevent it.
    NewtonianState state = stateOf({{5.0, -3.0, 2.0, 0.0}, {5.0, -3.0, 2.0, 0.0}}, 0.0);
    ASSERT_GT(state.epsilon, 0.0);
    physics::forces::Gravity::apply(state, 0.0);
    for (std::size_t i = 0; i < state.size(); i += 1) {
        EXPECT_TRUE(std::isfinite(state.forceX[i]) && std::isfinite(state.forceY[i]) && std::isfinite(state.forceZ[i]))
            << "body " << i;
        EXPECT_EQ(state.forceX[i], 0.0);
    }
}

TEST(Softening, CoincidentBodiesWithRadiiProduceFiniteForces)
{
    NewtonianState state = stateOf({{0.0, 0.0, 0.0, 1000.0}, {0.0, 0.0, 0.0, 500.0}}, 0.0);
    physics::forces::Gravity::apply(state, 0.0);
    for (std::size_t i = 0; i < state.size(); i += 1)
        EXPECT_TRUE(std::isfinite(state.forceX[i]) && std::isfinite(state.forceY[i]) && std::isfinite(state.forceZ[i]))
            << "body " << i;
}

TEST(Softening, SelfInteractionAndPaddingNeverProduceNaN)
{
    // Every body meets itself (d = 0) and the zero-mass padding slots in the SIMD loop.
    for (std::size_t n : {1u, 2u, 7u, 8u, 9u, 15u, 16u, 17u, 33u}) {
        for (const double radiusMax : {0.0, 1.0e3}) {
            NewtonianState state = stateOf(unit::randomCluster(n, static_cast<unsigned>(n), 1.0e6, 0.0, radiusMax), 0.0);
            physics::forces::Gravity::apply(state, 0.0);
            for (std::size_t i = 0; i < state.size(); i += 1)
                ASSERT_TRUE(std::isfinite(state.forceX[i]) && std::isfinite(state.forceY[i]) &&
                            std::isfinite(state.forceZ[i]))
                    << "n=" << n << " radiusMax=" << radiusMax << " body " << i;
        }
    }
}

TEST(Softening, DerivedEpsilonKeepsForcesSymmetric)
{
    // Equal masses: Newton's third law means the accelerations sum to zero.
    NewtonianState state = stateOf(unit::randomCluster(50, 3, 1.0e5, 100.0, 2000.0), 0.0);
    physics::forces::Gravity::apply(state, 0.0);
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    double scale = 0.0;
    for (std::size_t i = 0; i < state.size(); i += 1) {
        sumX += state.forceX[i];
        sumY += state.forceY[i];
        sumZ += state.forceZ[i];
        scale += std::abs(state.forceX[i]) + std::abs(state.forceY[i]) + std::abs(state.forceZ[i]);
    }
    EXPECT_LT((std::abs(sumX) + std::abs(sumY) + std::abs(sumZ)) / scale, 1e-12);
}
