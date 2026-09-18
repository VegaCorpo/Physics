#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <types/World.hpp>

namespace qa {

    /**
     * @brief Effective scalar mass, computed the same way the module does it
     * (float mantissa widened to double, times a double power of ten).
     */
    inline double scalarMass(const common::components::Mass& mass)
    {
        return static_cast<double>(mass.mantissa) * std::pow(10.0, mass.exponent);
    }

    struct Invariants {
            long double kinetic = 0;
            long double potential = 0;
            long double totalMass = 0;
            std::array<long double, 3> momentum{0, 0, 0};
            std::array<long double, 3> angularMomentum{0, 0, 0};
            std::array<long double, 3> centerOfMass{0, 0, 0};
            std::size_t nonFinite = 0;
    };

    /**
     * @brief Conserved quantities of a WorldState, accumulated in long double.
     *
     * The potential is the exact pairwise Newtonian potential (no softening),
     * so it is O(N^2); callers can skip it for large scenes.
     */
    inline Invariants computeInvariants(const common::WorldState& world, double G, bool withPotential)
    {
        Invariants inv;
        const std::size_t n = world.positions.size();

        for (std::size_t i = 0; i < n; i += 1) {
            const auto& p = world.positions[i];
            const auto& v = world.velocities[i];
            const long double m = scalarMass(world.mass[i]);

            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(v.x) ||
                !std::isfinite(v.y) || !std::isfinite(v.z)) {
                inv.nonFinite += 1;
                continue;
            }

            inv.totalMass += m;
            inv.kinetic += 0.5L * m * (v.x * (long double) v.x + v.y * (long double) v.y + v.z * (long double) v.z);
            inv.momentum[0] += m * v.x;
            inv.momentum[1] += m * v.y;
            inv.momentum[2] += m * v.z;
            inv.angularMomentum[0] += m * ((long double) p.y * v.z - (long double) p.z * v.y);
            inv.angularMomentum[1] += m * ((long double) p.z * v.x - (long double) p.x * v.z);
            inv.angularMomentum[2] += m * ((long double) p.x * v.y - (long double) p.y * v.x);
            inv.centerOfMass[0] += m * p.x;
            inv.centerOfMass[1] += m * p.y;
            inv.centerOfMass[2] += m * p.z;
        }

        if (inv.totalMass > 0) {
            for (auto& c : inv.centerOfMass)
                c /= inv.totalMass;
        }

        if (withPotential) {
            for (std::size_t i = 0; i < n; i += 1) {
                const long double mi = scalarMass(world.mass[i]);
                for (std::size_t j = i + 1; j < n; j += 1) {
                    const long double dx = (long double) world.positions[j].x - world.positions[i].x;
                    const long double dy = (long double) world.positions[j].y - world.positions[i].y;
                    const long double dz = (long double) world.positions[j].z - world.positions[i].z;
                    const long double r = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (r > 0)
                        inv.potential -= G * mi * scalarMass(world.mass[j]) / r;
                }
            }
        }
        return inv;
    }

} // namespace qa
