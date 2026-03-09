#include "Gravity.hpp"
#include "forces/Gravity.hpp"

void physics::events::ApplyPairGravityForce::apply(physics::events::PairGravityParams gravityParams)
{
    auto& corpsA = gravityParams.corpsA;
    auto& corpsB = gravityParams.corpsB;

    const auto disp = forces::Gravity::computeDisplacement(corpsA.position, corpsB.position);
    const auto invDist = forces::Gravity::computeInverseDistance(disp);
    const auto magnitude = forces::Gravity::G * corpsA.mass.value * corpsB.mass.value * invDist.invDistCubed;
    forces::Gravity::accumulateForce(corpsA.force, corpsB.force, disp, magnitude);
}
