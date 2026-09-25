#pragma once

#include <algorithm>
#include <boost/align/aligned_allocator.hpp>
#include <cmath>
#include <cstddef>
#include <experimental/simd>
#include <types/World.hpp>
#include <vector>

namespace physics {

    namespace stdx = std::experimental;

    inline constexpr std::size_t SIMD_WIDTH = stdx::native_simd<double>::size();

    inline constexpr std::size_t SIMD_ALIGNMENT = 64;

    template <typename T>
    using aligned_vector = std::vector<T, boost::alignment::aligned_allocator<T, SIMD_ALIGNMENT>>;

    inline double scalarMassOf(const common::components::Mass& masses)
    {
        return static_cast<double>(masses.mantissa) * std::pow(10.0, masses.exponent);
    }

    struct NewtonianState {
            static constexpr std::size_t BLOCK = 2 * SIMD_WIDTH;

            aligned_vector<double> posX;
            aligned_vector<double> posY;
            aligned_vector<double> posZ;
            aligned_vector<double> scalarMass;

            aligned_vector<double> forceX;
            aligned_vector<double> forceY;
            aligned_vector<double> forceZ;

            [[nodiscard]] std::size_t size() const noexcept { return _count; }

            [[nodiscard]] std::size_t paddedSize() const noexcept { return posX.size(); }

            [[nodiscard]] bool forcesStale() const noexcept { return _forces_stale; }

            void markForcesFresh() noexcept { this->_forces_stale = false; }

            void syncIn(const common::SpecificDataPhysics& world)
            {
                const std::size_t count = std::min(
                    {world.entitiesId.size(), world.positions.size(), world.velocities.size(), world.masses.size()});

                this->_resize(count);
                this->_trackEntities(world.entitiesId, count);

                for (std::size_t i = 0; i < count; i += 1) {
                    posX[i] = world.positions[i].x;
                    posY[i] = world.positions[i].y;
                    posZ[i] = world.positions[i].z;
                    scalarMass[i] = scalarMassOf(world.masses[i]);
                }
            }

            void syncOut(common::SpecificDataPhysics& world) const
            {
                const std::size_t count = std::min(this->_count, world.positions.size());

                for (std::size_t i = 0; i < count; i += 1) {
                    world.positions[i].x = posX[i];
                    world.positions[i].y = posY[i];
                    world.positions[i].z = posZ[i];
                }
            }

        private:
            void _resize(std::size_t count)
            {
                if (count == this->_count)
                    return;
                this->_count = count;

                const std::size_t padded = ((count + BLOCK - 1) / BLOCK) * BLOCK;

                posX.assign(padded, 0.0);
                posY.assign(padded, 0.0);
                posZ.assign(padded, 0.0);
                scalarMass.assign(padded, 0.0);
                forceX.assign(padded, 0.0);
                forceY.assign(padded, 0.0);
                forceZ.assign(padded, 0.0);
            }

            void _trackEntities(const std::vector<std::size_t>& entities, std::size_t count)
            {
                if (this->_entities.size() == count &&
                    std::equal(this->_entities.begin(), this->_entities.end(), entities.begin()))
                    return;

                this->_entities.assign(entities.begin(), entities.begin() + static_cast<std::ptrdiff_t>(count));
                this->_forces_stale = true;
            }

            std::size_t _count = 0;
            std::vector<std::size_t> _entities;
            bool _forces_stale = true;
    };
} // namespace physics
