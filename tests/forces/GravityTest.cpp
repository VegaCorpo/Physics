#include <entt/entt.hpp>
#include <gtest/gtest.h>
#include "components/GravityCache.hpp"
#include "components/kinematics/Position.hpp"
#include "components/properties/Mass.hpp"
#include "components/solver/ForceAccumulator.hpp"
#include "forces/Gravity.hpp"

class GravityTest : public ::testing::Test {
    protected:
        // computeScalarMass: mantissa * 10^exponent
        static physics::components::Mass makeMass(float mantissa, int exponent) { return {mantissa, exponent}; }
};

TEST_F(GravityTest, ComputeScalarMassCorrect)
{
    auto scalar = physics::forces::Gravity::computeScalarMass(makeMass(5.97f, 24));
    EXPECT_NEAR(scalar.value, 5.97e24, 1e18);
}

TEST_F(GravityTest, ComputeDisplacementCorrect)
{
    physics::components::Position a{0, 0, 0};
    physics::components::Position b{3, 4, 0};

    auto disp = physics::forces::Gravity::computeDisplacement(a, b);

    EXPECT_DOUBLE_EQ(disp.dx, 3.0);
    EXPECT_DOUBLE_EQ(disp.dy, 4.0);
    EXPECT_DOUBLE_EQ(disp.dz, 0.0);
}

TEST_F(GravityTest, InverseDistanceDecreaseWithRange)
{
    physics::components::Displacement near{1.0, 0, 0};
    physics::components::Displacement far{1000.0, 0, 0};

    auto invNear = physics::forces::Gravity::computeInverseDistance(near);
    auto invFar = physics::forces::Gravity::computeInverseDistance(far);

    EXPECT_GT(invNear.invDistCubed, invFar.invDistCubed);
}

TEST_F(GravityTest, EpsilonPreventsZeroDivision)
{
    physics::components::Displacement zero{0, 0, 0};

    EXPECT_NO_THROW({
        auto inv = physics::forces::Gravity::computeInverseDistance(zero);
        EXPECT_GT(inv.invDistCubed, 0.0);
    });
}

TEST_F(GravityTest, AccumulateForceIsSymmetric)
{
    physics::components::ForceAccumulator fa{0, 0, 0};
    physics::components::ForceAccumulator fb{0, 0, 0};
    physics::components::Displacement disp{1.0, 0, 0};

    physics::forces::Gravity::accumulateForce(fa, fb, disp, 100.0);

    // Newton 3 — action/réaction
    EXPECT_DOUBLE_EQ(fa.x, -fb.x);
    EXPECT_DOUBLE_EQ(fa.y, -fb.y);
    EXPECT_DOUBLE_EQ(fa.z, -fb.z);
}
