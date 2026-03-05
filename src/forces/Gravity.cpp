#include "Gravity.hpp"

//? Public methods

void physics::forces::Gravity::apply(entt::registry& registry, double /*dt*/)
{
    auto view =
        registry
            .view<physics::components::Position, physics::components::Mass, physics::components::ForceAccumulator>();

    for (auto entityA : view) {
        const auto& posA = view.get<components::Position>(entityA);
        const auto& massA = view.get<components::Mass>(entityA);
        auto& forceA = view.get<components::ForceAccumulator>(entityA);

        double m1 = computeMassValue(massA);

        for (auto entityB : view) {
            if (entityA >= entityB) {
                continue;
            }

            const auto& posB = view.get<components::Position>(entityB);
            const auto& massB = view.get<components::Mass>(entityB);
            auto& forceB = view.get<components::ForceAccumulator>(entityB);
            double m2 = computeMassValue(massB);

            applyPairwiseGravity(posA, posB, m1, m2, forceA, forceB);
        }
    }
}

//? Private methods

void physics::forces::Gravity::applyPairwiseGravity(const components::Position& posA, const components::Position& posB,
                                                    double m1, double m2, components::ForceAccumulator& forceA,
                                                    components::ForceAccumulator& forceB)
{
    double dx = posB.x - posA.x;
    double dy = posB.y - posA.y;
    double dz = posB.z - posA.z;

    double distanceSquared = dx * dx + dy * dy + dz * dz + EPSILON * EPSILON;
    double invDistance = 1.0 / std::sqrt(distanceSquared);
    double invDistanceCubed = invDistance * invDistance * invDistance;

    double forceMagnitude = computeForceMagnitude(m1, m2, invDistanceCubed);

    double fx = forceMagnitude * dx;
    double fy = forceMagnitude * dy;
    double fz = forceMagnitude * dz;

    forceA.x += fx;
    forceA.y += fy;
    forceA.z += fz;

    forceB.x -= fx;
    forceB.y -= fy;
    forceB.z -= fz;
}

double physics::forces::Gravity::computeMassValue(const components::Mass& mass)
{
    return mass.mantissa * std::pow(10.0, mass.exponent);
}

double physics::forces::Gravity::computeForceMagnitude(double m1, double m2, double invDistanceCubed)
{
    return G * m1 * m2 * invDistanceCubed;
}
