#pragma once

#include <cmath>
#include <cstddef>
#include <vector>
#include <types/World.hpp>
#include "Invariants.hpp"

namespace qa {

    /**
     * @brief High-precision reference integrator, independent from the module.
     *
     * Classical RK4 in long double (80-bit on x86-64) on the unsoftened
     * Newtonian N-body problem, sub-stepping every frame. With the default
     * sub-step count its global error is far below the module's double
     * precision Verlet error, so it can serve as ground truth for small scenes.
     */
    class Reference {
        public:
            Reference(const common::WorldState& world, long double G, long double softening2)
                : _G(G), _eps2(softening2)
            {
                const std::size_t n = world.positions.size();
                this->_x.resize(n);
                this->_y.resize(n);
                this->_z.resize(n);
                this->_vx.resize(n);
                this->_vy.resize(n);
                this->_vz.resize(n);
                this->_m.resize(n);
                for (std::size_t i = 0; i < n; i += 1) {
                    this->_x[i] = world.positions[i].x;
                    this->_y[i] = world.positions[i].y;
                    this->_z[i] = world.positions[i].z;
                    this->_vx[i] = world.velocities[i].x;
                    this->_vy[i] = world.velocities[i].y;
                    this->_vz[i] = world.velocities[i].z;
                    this->_m[i] = scalarMass(world.mass[i]);
                }
                this->_ax.resize(n);
                this->_ay.resize(n);
                this->_az.resize(n);
            }

            /// Advance by `dt` using `substeps` RK4 steps.
            void step(long double dt, std::size_t substeps)
            {
                const long double h = dt / static_cast<long double>(substeps);
                for (std::size_t s = 0; s < substeps; s += 1)
                    this->_rk4(h);
            }

            /// Write the current state back into a WorldState (positions and velocities).
            void writeTo(common::WorldState& world) const
            {
                for (std::size_t i = 0; i < this->_x.size(); i += 1) {
                    world.positions[i].x = static_cast<double>(this->_x[i]);
                    world.positions[i].y = static_cast<double>(this->_y[i]);
                    world.positions[i].z = static_cast<double>(this->_z[i]);
                    world.velocities[i].x = static_cast<double>(this->_vx[i]);
                    world.velocities[i].y = static_cast<double>(this->_vy[i]);
                    world.velocities[i].z = static_cast<double>(this->_vz[i]);
                }
            }

        private:
            using Vec = std::vector<long double>;

            void _acceleration(const Vec& x, const Vec& y, const Vec& z, Vec& ax, Vec& ay, Vec& az) const
            {
                const std::size_t n = x.size();
                for (std::size_t i = 0; i < n; i += 1) {
                    long double sx = 0, sy = 0, sz = 0;
                    for (std::size_t j = 0; j < n; j += 1) {
                        if (i == j)
                            continue;
                        const long double dx = x[j] - x[i];
                        const long double dy = y[j] - y[i];
                        const long double dz = z[j] - z[i];
                        const long double r2 = dx * dx + dy * dy + dz * dz + this->_eps2;
                        const long double inv = 1.0L / std::sqrt(r2);
                        const long double mag = this->_G * this->_m[j] * inv * inv * inv;
                        sx += mag * dx;
                        sy += mag * dy;
                        sz += mag * dz;
                    }
                    ax[i] = sx;
                    ay[i] = sy;
                    az[i] = sz;
                }
            }

            void _rk4(long double h)
            {
                const std::size_t n = this->_x.size();
                Vec kx1(n), ky1(n), kz1(n), kvx1(n), kvy1(n), kvz1(n);
                Vec kx2(n), ky2(n), kz2(n), kvx2(n), kvy2(n), kvz2(n);
                Vec kx3(n), ky3(n), kz3(n), kvx3(n), kvy3(n), kvz3(n);
                Vec kx4(n), ky4(n), kz4(n), kvx4(n), kvy4(n), kvz4(n);
                Vec tx(n), ty(n), tz(n), tvx(n), tvy(n), tvz(n);

                // k1
                this->_acceleration(this->_x, this->_y, this->_z, kvx1, kvy1, kvz1);
                for (std::size_t i = 0; i < n; i += 1) {
                    kx1[i] = this->_vx[i];
                    ky1[i] = this->_vy[i];
                    kz1[i] = this->_vz[i];
                    tx[i] = this->_x[i] + 0.5L * h * kx1[i];
                    ty[i] = this->_y[i] + 0.5L * h * ky1[i];
                    tz[i] = this->_z[i] + 0.5L * h * kz1[i];
                    tvx[i] = this->_vx[i] + 0.5L * h * kvx1[i];
                    tvy[i] = this->_vy[i] + 0.5L * h * kvy1[i];
                    tvz[i] = this->_vz[i] + 0.5L * h * kvz1[i];
                }
                // k2
                this->_acceleration(tx, ty, tz, kvx2, kvy2, kvz2);
                for (std::size_t i = 0; i < n; i += 1) {
                    kx2[i] = tvx[i];
                    ky2[i] = tvy[i];
                    kz2[i] = tvz[i];
                    tx[i] = this->_x[i] + 0.5L * h * kx2[i];
                    ty[i] = this->_y[i] + 0.5L * h * ky2[i];
                    tz[i] = this->_z[i] + 0.5L * h * kz2[i];
                    tvx[i] = this->_vx[i] + 0.5L * h * kvx2[i];
                    tvy[i] = this->_vy[i] + 0.5L * h * kvy2[i];
                    tvz[i] = this->_vz[i] + 0.5L * h * kvz2[i];
                }
                // k3
                this->_acceleration(tx, ty, tz, kvx3, kvy3, kvz3);
                for (std::size_t i = 0; i < n; i += 1) {
                    kx3[i] = tvx[i];
                    ky3[i] = tvy[i];
                    kz3[i] = tvz[i];
                    tx[i] = this->_x[i] + h * kx3[i];
                    ty[i] = this->_y[i] + h * ky3[i];
                    tz[i] = this->_z[i] + h * kz3[i];
                    tvx[i] = this->_vx[i] + h * kvx3[i];
                    tvy[i] = this->_vy[i] + h * kvy3[i];
                    tvz[i] = this->_vz[i] + h * kvz3[i];
                }
                // k4
                this->_acceleration(tx, ty, tz, kvx4, kvy4, kvz4);
                for (std::size_t i = 0; i < n; i += 1) {
                    kx4[i] = tvx[i];
                    ky4[i] = tvy[i];
                    kz4[i] = tvz[i];
                }

                const long double w = h / 6.0L;
                for (std::size_t i = 0; i < n; i += 1) {
                    this->_x[i] += w * (kx1[i] + 2 * kx2[i] + 2 * kx3[i] + kx4[i]);
                    this->_y[i] += w * (ky1[i] + 2 * ky2[i] + 2 * ky3[i] + ky4[i]);
                    this->_z[i] += w * (kz1[i] + 2 * kz2[i] + 2 * kz3[i] + kz4[i]);
                    this->_vx[i] += w * (kvx1[i] + 2 * kvx2[i] + 2 * kvx3[i] + kvx4[i]);
                    this->_vy[i] += w * (kvy1[i] + 2 * kvy2[i] + 2 * kvy3[i] + kvy4[i]);
                    this->_vz[i] += w * (kvz1[i] + 2 * kvz2[i] + 2 * kvz3[i] + kvz4[i]);
                }
            }

            long double _G;
            long double _eps2;
            Vec _x, _y, _z, _vx, _vy, _vz, _m;
            Vec _ax, _ay, _az;
    };

} // namespace qa
