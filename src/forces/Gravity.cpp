#include "Gravity.hpp"
#include <algorithm>
#include <boost/iterator/counting_iterator.hpp>
#include <cmath>
#include <cstddef>
#include <execution>
#include <experimental/simd>

namespace {
    using simd_t = physics::stdx::native_simd<double>;

    constexpr std::size_t LANES = physics::SIMD_WIDTH;
} // namespace

//? Public methods

void physics::forces::Gravity::apply(const common::WorldState& world, NewtonianState& state, double /*dt*/)
{
    state.syncIn(world);

    if (state.size() == 0)
        return;

    physics::forces::Gravity::_computeGravity(state);
}

//? Private methods

void physics::forces::Gravity::_computeGravity(NewtonianState& state)
{
    const std::size_t count = state.size();
    const std::size_t padded = state.paddedSize();

    const double* __restrict posX = state.posX.data();
    const double* __restrict posY = state.posY.data();
    const double* __restrict posZ = state.posZ.data();
    const double* __restrict mass = state.scalarMass.data();

    double* __restrict forceX = state.forceX.data();
    double* __restrict forceY = state.forceY.data();
    double* __restrict forceZ = state.forceZ.data();

    std::for_each(std::execution::par_unseq, boost::counting_iterator<std::size_t>(0),
                  boost::counting_iterator<std::size_t>(count),
                  [=](std::size_t i)
                  {
                      const simd_t myPx = posX[i];
                      const simd_t myPy = posY[i];
                      const simd_t myPz = posZ[i];
                      const simd_t myMassG = mass[i] * G;

                      simd_t accFx0 = 0.0;
                      simd_t accFy0 = 0.0;
                      simd_t accFz0 = 0.0;
                      simd_t accFx1 = 0.0;
                      simd_t accFy1 = 0.0;
                      simd_t accFz1 = 0.0;

                      const auto accumulate = [&](std::size_t j, simd_t& accFx, simd_t& accFy, simd_t& accFz)
                      {
                          const simd_t dx = simd_t(&posX[j], stdx::vector_aligned) - myPx;
                          const simd_t dy = simd_t(&posY[j], stdx::vector_aligned) - myPy;
                          const simd_t dz = simd_t(&posZ[j], stdx::vector_aligned) - myPz;
                          const simd_t massJ(&mass[j], stdx::vector_aligned);

                          const simd_t r2 = dx * dx + dy * dy + dz * dz + EPSILON2;
                          const simd_t invDist = 1.0 / stdx::sqrt(r2);
                          const simd_t mag = myMassG * massJ * invDist * invDist * invDist;

                          accFx += mag * dx;
                          accFy += mag * dy;
                          accFz += mag * dz;
                      };

                      for (std::size_t j = 0; j < padded; j += NewtonianState::BLOCK) {
                          accumulate(j, accFx0, accFy0, accFz0);
                          accumulate(j + LANES, accFx1, accFy1, accFz1);
                      }

                      forceX[i] = stdx::reduce(accFx0 + accFx1);
                      forceY[i] = stdx::reduce(accFy0 + accFy1);
                      forceZ[i] = stdx::reduce(accFz0 + accFz1);
                  });
}

physics::components::ScalarMass physics::forces::Gravity::computeScalarMass(const common::components::Mass& mass)
{
    return {physics::scalarMassOf(mass)};
}

physics::components::InverseDistance
physics::forces::Gravity::computeInverseDistance(const physics::components::Displacement& disp)
{
    double r2 = disp.dx * disp.dx + disp.dy * disp.dy + disp.dz * disp.dz + EPSILON2;
    double invDist = 1.0 / std::sqrt(r2);

    return {invDist * invDist * invDist};
}
