#include "Gravity.hpp"

//? Public methods

void physics::forces::Gravity::apply(entt::registry& registry, double /*dt*/)
{
    registry.view<components::Mass>().each(
        [&](auto entity, const auto& m)
        { registry.emplace_or_replace<components::ScalarMass>(entity, computeScalarMass(m)); });

    auto view = registry.view<components::Position, components::ScalarMass, components::ForceAccumulator>();

    auto& posPool = registry.storage<components::Position>();
    auto& massPool = registry.storage<components::ScalarMass>();
    auto& forcePool = registry.storage<components::ForceAccumulator>();

    for (auto it = view.begin(); it != view.end(); ++it) {
        const auto entityA = *it;
        const auto& posA = posPool.get(entityA);
        const auto& mA = massPool.get(entityA);
        auto& forceA = forcePool.get(entityA);

        for (auto jt = std::next(it); jt != view.end(); ++jt) {
            const auto entityB = *jt;
            const auto& posB = posPool.get(entityB);
            const auto& mB = massPool.get(entityB);
            auto& forceB = forcePool.get(entityB);

            const auto disp = computeDisplacement(posA, posB);
            const auto invDist = computeInverseDistance(disp);
            const auto magnitude = G * mA.value * mB.value * invDist.invDistCubed;
            accumulateForce(forceA, forceB, disp, magnitude);
        }
    }
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
    double r2 = disp.dx * disp.dx + disp.dy * disp.dy + disp.dz * disp.dz + EPSILON * EPSILON;
    double invDist = 1.0 / std::sqrt(r2);

    return {invDist * invDist * invDist};
}

physics::components::ScalarMass physics::forces::Gravity::computeScalarMass(const physics::components::Mass& mass)
{
    return {mass.mantissa * std::pow(10.0, mass.exponent)};
}
