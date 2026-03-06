#pragma once

#include <entt/entity/fwd.hpp>
#include <entt/entity/view.hpp>
#include "components/GravityCache.hpp"
#include "components/kinematics/Position.hpp"
#include "components/solver/ForceAccumulator.hpp"

namespace physics::events {

    struct GravityParams {
            components::ScalarMass mass;
            components::Position position;
            components::ForceAccumulator force;
    };

    struct PairGravityParams {
            GravityParams corpsA;
            GravityParams corpsB;
    };

    class ApplyPairGravityForce {
        public:
            static void apply(PairGravityParams gravityParams);

        private:
    };
} // namespace physics::events
