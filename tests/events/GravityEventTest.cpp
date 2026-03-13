#include <entt/entt.hpp>
#include <gtest/gtest.h>
#include "components/GravityCache.hpp"
#include "components/kinematics/Position.hpp"
#include "components/solver/ForceAccumulator.hpp"
#include "events/Gravity.hpp"
#include "forces/Gravity.hpp"

TEST(GravityEventTest, ApplyPairAccumulatesForceOnBothBodies)
{
    physics::components::ScalarMass massA{1e10};
    physics::components::ScalarMass massB{1e10};
    physics::components::Position posA{0.0, 0.0, 0.0};
    physics::components::Position posB{1000.0, 0.0, 0.0};
    physics::components::ForceAccumulator forceA{0.0, 0.0, 0.0};
    physics::components::ForceAccumulator forceB{0.0, 0.0, 0.0};

    physics::events::PairGravityParams params{{&massA, &posA, &forceA}, {&massB, &posB, &forceB}};

    physics::events::ApplyPairGravityForce::apply(params);

    // Les deux corps reçoivent une force non nulle
    EXPECT_NE(forceA.x, 0.0);
    EXPECT_NE(forceB.x, 0.0);
}

TEST(GravityEventTest, ApplyPairRespectNewtonThirdLaw)
{
    physics::components::ScalarMass massA{1e10};
    physics::components::ScalarMass massB{2e10};
    physics::components::Position posA{0.0, 0.0, 0.0};
    physics::components::Position posB{500.0, 0.0, 0.0};
    physics::components::ForceAccumulator forceA{0.0, 0.0, 0.0};
    physics::components::ForceAccumulator forceB{0.0, 0.0, 0.0};

    physics::events::PairGravityParams params{{&massA, &posA, &forceA}, {&massB, &posB, &forceB}};

    physics::events::ApplyPairGravityForce::apply(params);

    EXPECT_DOUBLE_EQ(forceA.x, -forceB.x);
    EXPECT_DOUBLE_EQ(forceA.y, -forceB.y);
    EXPECT_DOUBLE_EQ(forceA.z, -forceB.z);
}

TEST(GravityEventTest, ApplyEnqueuesCorrectPairCountForThreeBodies)
{
    entt::registry registry;
    entt::dispatcher dispatcher;

    for (int i = 0; i < 3; i++) {
        auto e = registry.create();
        registry.emplace<physics::components::Position>(e, i * 1000.0, 0.0, 0.0);
        registry.emplace<physics::components::ScalarMass>(e, 1e10);
        registry.emplace<physics::components::ForceAccumulator>(e, 0.0, 0.0, 0.0);
    }

    int count = 0;
    dispatcher.sink<physics::events::PairGravityParams>()
        .connect<[](void* payload, physics::events::PairGravityParams) { (*static_cast<int*>(payload))++; }>(
            static_cast<void*>(&count));

    physics::forces::Gravity::apply(registry, dispatcher, 0.0);
    dispatcher.update<physics::events::PairGravityParams>();

    // N=3 → N*(N-1)/2 = 3 paires
    EXPECT_EQ(count, 3);
}
