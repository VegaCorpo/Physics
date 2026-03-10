#include "Gravity.hpp"
#include <entt/signal/fwd.hpp>
#include "components/GravityCache.hpp"
#include "components/kinematics/Position.hpp"
#include "components/properties/Mass.hpp"
#include "components/solver/ForceAccumulator.hpp"
#include "events/Gravity.hpp"

//? Public methods

void physics::forces::Gravity::apply(entt::registry& registry, entt::dispatcher& dispatcher, double /*dt*/)
{
    auto view = registry.view<components::Position, components::ScalarMass, components::ForceAccumulator>();

    for (auto entityA : view) {
        events::GravityParams paramsA{.mass = &view.get<components::ScalarMass>(entityA),
                                      .position = &view.get<components::Position>(entityA),
                                      .force = &view.get<components::ForceAccumulator>(entityA)};

        for (auto entityB : view) {
            if (entityB <= entityA)
                continue;
            events::GravityParams paramsB{.mass = &view.get<components::ScalarMass>(entityB),
                                          .position = &view.get<components::Position>(entityB),
                                          .force = &view.get<components::ForceAccumulator>(entityB)};

            dispatcher.enqueue<events::PairGravityParams>({paramsA, paramsB});
        }
    }
}

physics::components::ScalarMass physics::forces::Gravity::computeScalarMass(const physics::components::Mass& mass)
{
    return {mass.mantissa * std::pow(10.0, mass.exponent)};
}

//? Private methods

void physics::forces::Gravity::accumulateForce(components::ForceAccumulator& forceA,
                                               components::ForceAccumulator& forceB,
                                               const components::Displacement& disp, double magnitude)
{
    double fx = magnitude * disp.dx;
    double fy = magnitude * disp.dy;
    double fz = magnitude * disp.dz;

    forceA.x += fx;
    forceA.y += fy;
    forceA.z += fz;

    forceB.x -= fx;
    forceB.y -= fy;
    forceB.z -= fz;
}

physics::components::Displacement
physics::forces::Gravity::computeDisplacement(const physics::components::Position& posA,
                                              const physics::components::Position& posB)
{
    return {posB.x - posA.x, posB.y - posA.y, posB.z - posA.z};
}

physics::components::InverseDistance
physics::forces::Gravity::computeInverseDistance(const physics::components::Displacement& disp)
{
    double r2 =
        disp.dx * disp.dx + disp.dy * disp.dy + disp.dz * disp.dz + forces::Gravity::EPSILON * forces::Gravity::EPSILON;
    double invDist = 1.0 / std::sqrt(r2);

    return {invDist * invDist * invDist};
}
