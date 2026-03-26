#include "Gravity.hpp"
#include <algorithm>
#include <boost/iterator/counting_iterator.hpp>
#include <cstddef>
#include <entt/entity/fwd.hpp>
#include <entt/signal/fwd.hpp>
#include <execution>
#include <iostream>
#include <memory>
#include <numeric>
#include <oneapi/tbb/parallel_for_each.h>
#include <oneapi/tbb/parallel_reduce.h>
#include "components/GravityCache.hpp"
#include "components/kinematics/Position.hpp"
#include "components/properties/Mass.hpp"
#include "components/solver/ForceAccumulator.hpp"

//? Public methods
void physics::forces::Gravity::apply(entt::registry& registry, entt::dispatcher& dispatcher, double /*dt*/)
{
    auto view = registry.group<components::PositionX, components::PositionY, components::PositionZ,
                               components::ScalarMass, components::ForceAccumulator>();
    const size_t count = view.size();

    _computeGravity(registry, count);
}

//? Private methods

void physics::forces::Gravity::_computeGravity(entt::registry& registry, const size_t count)
{
    auto group = registry.group<components::PositionX, components::PositionY, components::PositionZ,
                                components::ScalarMass, components::ForceAccumulator>();

    const components::PositionX* __restrict posX = *group.storage<components::PositionX>()->raw();
    const components::PositionY* __restrict posY = *group.storage<components::PositionY>()->raw();
    const components::PositionZ* __restrict posZ = *group.storage<components::PositionZ>()->raw();
    const components::ScalarMass* __restrict mass = *group.storage<components::ScalarMass>()->raw();

    std::for_each(
        std::execution::par_unseq, boost::counting_iterator<size_t>(0), boost::counting_iterator<size_t>(count),
        [=](size_t i)
        {
            components::ForceAccumulator* __restrict forceAcc = *group.storage<components::ForceAccumulator>()->raw();
            const double myPx = posX[i].value;
            const double myPy = posY[i].value;
            const double myPz = posZ[i].value;
            const double myMassG = mass[i].value * G;

            double accFx = 0.0;
            double accFy = 0.0;
            double accFz = 0.0;

            for (size_t j = 0; j < count; j += 1) {
                double dx = posX[j].value - myPx;
                double dy = posY[j].value - myPy;
                double dz = posZ[j].value - myPz;

                double r2 = dx * dx + dy * dy + dz * dz + EPSILON2;
                double invDist = 1 / std::sqrt(r2);
                double invDist3 = invDist * invDist * invDist;
                double mag = myMassG * mass[j].value * invDist3;

                accFx += mag * dx;
                accFy += mag * dy;
                accFz += mag * dz;
            };

            forceAcc[i].x = accFx;
            forceAcc[i].y = accFy;
            forceAcc[i].z = accFz;
        });
}

physics::components::ScalarMass physics::forces::Gravity::computeScalarMass(const physics::components::Mass& mass)
{
    return {mass.mantissa * std::pow(10.0, mass.exponent)};
}

physics::components::InverseDistance
physics::forces::Gravity::computeInverseDistance(const physics::components::Displacement& disp)
{
    double r2 = disp.dx * disp.dx + disp.dy * disp.dy + disp.dz * disp.dz + EPSILON2;
    double invDist = 1.0 / std::sqrt(r2);

    return {invDist * invDist * invDist};
}
