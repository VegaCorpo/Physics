#include <entt/entt.hpp>
#include <gtest/gtest.h>
#include "components/GravityCache.hpp"
#include "components/kinematics/Acceleration.hpp"
#include "components/kinematics/Position.hpp"
#include "components/kinematics/Velocity.hpp"
#include "components/solver/ForceAccumulator.hpp"
#include "components/solver/PreviousAcceleration.hpp"
#include "integration/Verlet.hpp"

class VerletTest : public ::testing::Test {
    protected:
        entt::registry registry;

        entt::entity createBody(double mass, physics::components::Position pos,
                                physics::components::Velocity vel = {0.0, 0.0, 0.0},
                                physics::components::Acceleration acc = {0.0, 0.0, 0.0})
        {
            auto entity = registry.create();
            registry.emplace<physics::components::Position>(entity, pos.x, pos.y, pos.z);
            registry.emplace<physics::components::Velocity>(entity, vel.x, vel.y, vel.z);
            registry.emplace<physics::components::Acceleration>(entity, acc.x, acc.y, acc.z);
            registry.emplace<physics::components::PreviousAcceleration>(entity, acc.x, acc.y, acc.z);
            registry.emplace<physics::components::ScalarMass>(entity, mass);
            registry.emplace<physics::components::ForceAccumulator>(entity, 0.0, 0.0, 0.0);
            return entity;
        }
};

// Avec force nulle, la position avance selon la vitesse initiale
TEST_F(VerletTest, ConstantVelocityNoForce)
{
    auto e = createBody(1.0, {0, 0, 0}, {1.0, 0, 0});
    double dt = 1.0;

    physics::integration::Verlet::integrate(registry, dt);

    auto& pos = registry.get<physics::components::Position>(e);
    EXPECT_DOUBLE_EQ(pos.x, 1.0);
    EXPECT_DOUBLE_EQ(pos.y, 0.0);
    EXPECT_DOUBLE_EQ(pos.z, 0.0);
}

// Chute libre: x(t) = 0.5 * a * dt^2 depuis le repos
TEST_F(VerletTest, ConstantForceFreeFall)
{
    auto e = createBody(1.0, {0, 0, 0});

    double dt = 1.0;
    double force = -9.81;

    // Step 1: force → acc calculée, position utilise prevAcc=0
    registry.get<physics::components::ForceAccumulator>(e) = {0.0, force, 0.0};
    physics::integration::Verlet::integrate(registry, dt);

    // Step 2: force constante → on remet la force, maintenant prevAcc = -9.81
    registry.get<physics::components::ForceAccumulator>(e) = {0.0, force, 0.0};
    physics::integration::Verlet::integrate(registry, dt);

    // Après 2 steps avec accélération constante: y = a*dt^2 (formule Verlet)
    auto& pos = registry.get<physics::components::Position>(e);
    EXPECT_NEAR(pos.y, force * dt * dt, 1e-9);
}

// Masse nulle → accélération reste zéro, pas de NaN
TEST_F(VerletTest, ZeroMassNoNaN)
{
    auto e = createBody(0.0, {1, 2, 3}, {0, 0, 0});
    registry.get<physics::components::ForceAccumulator>(e) = {999.0, 999.0, 999.0};

    EXPECT_NO_THROW(physics::integration::Verlet::integrate(registry, 1.0));

    auto& acc = registry.get<physics::components::Acceleration>(e);
    EXPECT_DOUBLE_EQ(acc.x, 0.0);
}

// La force est remise à zéro après intégration
TEST_F(VerletTest, ForceAccumulatorResetAfterIntegration)
{
    auto e = createBody(1.0, {0, 0, 0});
    registry.get<physics::components::ForceAccumulator>(e) = {5.0, 3.0, 1.0};

    physics::integration::Verlet::integrate(registry, 0.1);

    auto& force = registry.get<physics::components::ForceAccumulator>(e);
    EXPECT_DOUBLE_EQ(force.x, 0.0);
    EXPECT_DOUBLE_EQ(force.y, 0.0);
    EXPECT_DOUBLE_EQ(force.z, 0.0);
}
