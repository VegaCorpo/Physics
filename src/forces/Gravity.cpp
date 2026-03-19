#include "Gravity.hpp"
#include <algorithm>
#include <boost/iterator/counting_iterator.hpp>
#include <cstddef>
#include <entt/entity/fwd.hpp>
#include <entt/signal/fwd.hpp>
#include <execution>
#include <oneapi/tbb/parallel_for_each.h>
#include "components/GravityCache.hpp"
#include "components/kinematics/Position.hpp"
#include "components/properties/Mass.hpp"
#include "components/solver/ForceAccumulator.hpp"
#include "events/Gravity.hpp"

//? Public methods
void physics::forces::Gravity::apply(entt::registry& registry, entt::dispatcher& dispatcher, double /*dt*/)
{
    auto view = registry.view<components::Position, components::ScalarMass, components::ForceAccumulator>();
    const size_t estimatedCount = view.size_hint();

    _initVectors(estimatedCount);

    view.each(
        [&](entt::entity entity, const components::Position& pos, const components::ScalarMass& mass,
            const components::ForceAccumulator&)
        {
            _entities.push_back(entity);
            _posX.push_back(pos.x);
            _posY.push_back(pos.y);
            _posZ.push_back(pos.z);
            _mass.push_back(mass.value);
        });

    const size_t count = _entities.size();

    _computeGravity(count);

    _accumulateForce(registry, count);
}

//? Private methods

void physics::forces::Gravity::_initVectors(const size_t estimatedCount)
{
    _entities.clear();
    _posX.clear();
    _posY.clear();
    _posZ.clear();
    _mass.clear();

    _entities.reserve(estimatedCount);
    _posX.reserve(estimatedCount);
    _posY.reserve(estimatedCount);
    _posZ.reserve(estimatedCount);
    _mass.reserve(estimatedCount);

    _outForceX.assign(estimatedCount, 0.0);
    _outForceY.assign(estimatedCount, 0.0);
    _outForceZ.assign(estimatedCount, 0.0);
}

void physics::forces::Gravity::_computeGravity(const size_t count)
{
    const double* posX = _posX.data();
    const double* posY = _posY.data();
    const double* posZ = _posZ.data();
    const double* mass = _mass.data();
    double* forceX = _outForceX.data();
    double* forceY = _outForceY.data();
    double* forceZ = _outForceZ.data();

    std::for_each(std::execution::par_unseq, boost::counting_iterator<size_t>(0),
                  boost::counting_iterator<size_t>(count),
                  [=](size_t i)
                  {
                      double accFx = 0.0, accFy = 0.0, accFz = 0.0;
                      const double myPx = posX[i], myPy = posY[i], myPz = posZ[i], myMass = mass[i];

                      for (size_t j = 0; j < count; ++j) {
                          double dx = posX[j] - myPx;
                          double dy = posY[j] - myPy;
                          double dz = posZ[j] - myPz;

                          double r2 = dx * dx + dy * dy + dz * dz + EPSILON * EPSILON;
                          double invDist = 1.0 / std::sqrt(r2);
                          double mag = G * myMass * mass[j] * (invDist * invDist * invDist);

                          accFx += mag * dx;
                          accFy += mag * dy;
                          accFz += mag * dz;
                      }
                      forceX[i] = accFx;
                      forceY[i] = accFy;
                      forceZ[i] = accFz;
                  });
}

void physics::forces::Gravity::_accumulateForce(entt::registry& registry, const size_t count)
{
    double* fx = _outForceX.data();
    double* fy = _outForceY.data();
    double* fz = _outForceZ.data();

    for (size_t i = 0; i < count; ++i) {
        auto& force = registry.get<components::ForceAccumulator>(_entities[i]);
        force.x += fx[i];
        force.y += fy[i];
        force.z += fz[i];
    }
}

physics::components::ScalarMass physics::forces::Gravity::computeScalarMass(const physics::components::Mass& mass)
{
    return {mass.mantissa * std::pow(10.0, mass.exponent)};
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
