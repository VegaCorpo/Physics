#include <components/acceleration.hpp>
#include <components/mass.hpp>
#include <components/position.hpp>
#include <components/velocity.hpp>
#include <entt/entt.hpp>
#include <gtest/gtest.h>
#include "components/GravityCache.hpp"
#include "components/kinematics/Position.hpp"
#include "components/kinematics/Velocity.hpp"
#include "components/properties/Mass.hpp"
#include "components/solver/ForceAccumulator.hpp"
#include "components/solver/PreviousAcceleration.hpp"
#include "NewtonianPhysics.hpp"

class NewtonianPhysicsTest : public ::testing::Test {
    protected:
        entt::registry registry;
        entt::dispatcher dispatcher;

        entt::entity createCoreBody(float px, float py, float pz, float vx, float vy, float vz, float mantissa,
                                    int exponent)
        {
            auto e = registry.create();
            registry.emplace<::components::Position>(e, px, py, pz);
            registry.emplace<::components::Velocity>(e, vx, vy, vz);
            registry.emplace<::components::Acceleration>(e, 0.0f, 0.0f, 0.0f);
            registry.emplace<::components::Mass>(e, mantissa, exponent);
            return e;
        }
};

// --- init ---

TEST_F(NewtonianPhysicsTest, InitDoesNotCrash)
{
    createCoreBody(0, 0, 0, 0, 0, 0, 5.97f, 24);
    EXPECT_NO_THROW(physics::NewtonianPhysics::init(registry, dispatcher));
}

TEST_F(NewtonianPhysicsTest, InitCreatesScalarMass)
{
    auto e = createCoreBody(0, 0, 0, 0, 0, 0, 2.0f, 3);
    physics::NewtonianPhysics::init(registry, dispatcher);

    ASSERT_TRUE(registry.all_of<physics::components::ScalarMass>(e));
    auto& scalar = registry.get<physics::components::ScalarMass>(e);
    EXPECT_NEAR(scalar.value, 2.0 * std::pow(10.0, 3), 1e-6);
}

TEST_F(NewtonianPhysicsTest, InitCreatesForceAccumulator)
{
    auto e = createCoreBody(0, 0, 0, 0, 0, 0, 1.0f, 0);
    physics::NewtonianPhysics::init(registry, dispatcher);

    EXPECT_TRUE(registry.all_of<physics::components::ForceAccumulator>(e));
}

TEST_F(NewtonianPhysicsTest, InitCreatesPreviousAcceleration)
{
    auto e = createCoreBody(0, 0, 0, 0, 0, 0, 1.0f, 0);
    physics::NewtonianPhysics::init(registry, dispatcher);

    EXPECT_TRUE(registry.all_of<physics::components::PreviousAcceleration>(e));
}

// --- syncIn ---

TEST_F(NewtonianPhysicsTest, SyncInCopiesPositionToPhysics)
{
    auto e = createCoreBody(1.0f, 2.0f, 3.0f, 0, 0, 0, 1.0f, 0);
    physics::NewtonianPhysics::syncIn(registry);

    ASSERT_TRUE(registry.all_of<physics::components::Position>(e));
    auto& pos = registry.get<physics::components::Position>(e);
    EXPECT_DOUBLE_EQ(pos.x, 1.0);
    EXPECT_DOUBLE_EQ(pos.y, 2.0);
    EXPECT_DOUBLE_EQ(pos.z, 3.0);
}

TEST_F(NewtonianPhysicsTest, SyncInCopiesVelocityToPhysics)
{
    auto e = createCoreBody(0, 0, 0, 4.0f, 5.0f, 6.0f, 1.0f, 0);
    physics::NewtonianPhysics::syncIn(registry);

    ASSERT_TRUE(registry.all_of<physics::components::Velocity>(e));
    auto& vel = registry.get<physics::components::Velocity>(e);
    EXPECT_DOUBLE_EQ(vel.x, 4.0);
    EXPECT_DOUBLE_EQ(vel.y, 5.0);
    EXPECT_DOUBLE_EQ(vel.z, 6.0);
}

TEST_F(NewtonianPhysicsTest, SyncInCopiesMassToPhysics)
{
    auto e = createCoreBody(0, 0, 0, 0, 0, 0, 3.0f, 7);
    physics::NewtonianPhysics::syncIn(registry);

    ASSERT_TRUE(registry.all_of<physics::components::Mass>(e));
    auto& mass = registry.get<physics::components::Mass>(e);
    EXPECT_FLOAT_EQ(mass.mantissa, 3.0f);
    EXPECT_EQ(mass.exponent, 7);
}

// --- syncOut ---

TEST_F(NewtonianPhysicsTest, SyncOutCopiesPositionToCore)
{
    auto e = registry.create();
    registry.emplace<physics::components::Position>(e, 7.0, 8.0, 9.0);

    physics::NewtonianPhysics::syncOut(registry);

    ASSERT_TRUE(registry.all_of<::components::Position>(e));
    auto& pos = registry.get<::components::Position>(e);
    EXPECT_FLOAT_EQ(pos.x, 7.0f);
    EXPECT_FLOAT_EQ(pos.y, 8.0f);
    EXPECT_FLOAT_EQ(pos.z, 9.0f);
}

TEST_F(NewtonianPhysicsTest, SyncOutCopiesVelocityToCore)
{
    auto e = registry.create();
    registry.emplace<physics::components::Velocity>(e, 1.0, 2.0, 3.0);

    physics::NewtonianPhysics::syncOut(registry);

    ASSERT_TRUE(registry.all_of<::components::Velocity>(e));
    auto& vel = registry.get<::components::Velocity>(e);
    EXPECT_FLOAT_EQ(vel.x, 1.0f);
    EXPECT_FLOAT_EQ(vel.y, 2.0f);
    EXPECT_FLOAT_EQ(vel.z, 3.0f);
}

// --- update ---

TEST_F(NewtonianPhysicsTest, UpdateDoesNotCrash)
{
    createCoreBody(0, 0, 0, 0, 0, 0, 5.97f, 24);
    physics::NewtonianPhysics::init(registry, dispatcher);
    EXPECT_NO_THROW(physics::NewtonianPhysics::update(registry, dispatcher, 0.1));
}

TEST_F(NewtonianPhysicsTest, UpdateMovesBodyWithInitialVelocity)
{
    auto e = createCoreBody(0, 0, 0, 1.0f, 0, 0, 1.0f, 0);
    physics::NewtonianPhysics::init(registry, dispatcher);
    physics::NewtonianPhysics::update(registry, dispatcher, 1.0);

    // syncOut recopie dans ::components::Position
    auto& pos = registry.get<::components::Position>(e);
    EXPECT_GT(pos.x, 0.0f);
}

// --- shutdown ---

TEST_F(NewtonianPhysicsTest, ShutdownClearsPhysicsComponents)
{
    auto e = createCoreBody(0, 0, 0, 0, 0, 0, 1.0f, 0);
    physics::NewtonianPhysics::init(registry, dispatcher);
    physics::NewtonianPhysics::shutdown(registry);

    EXPECT_FALSE(registry.all_of<physics::components::Position>(e));
    EXPECT_FALSE(registry.all_of<physics::components::Velocity>(e));
    EXPECT_FALSE(registry.all_of<physics::components::ForceAccumulator>(e));
}
