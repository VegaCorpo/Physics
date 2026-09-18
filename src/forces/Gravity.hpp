#pragma once

#include <cstdint>
#include <types/World.hpp>
#include <vector>
#include "components/gravity_cache/GravityCache.hpp"
#include "components/NewtonianState.hpp"
#include "spatial/Octree.hpp"

namespace physics::forces {

    enum class GravityMode {
        BruteForce,
        BarnesHut,
    };

    constexpr double G = 6.67430e-20; // Gravitational constant
    constexpr double EPSILON = 1e-6; // Small value to prevent division by zero
    constexpr double EPSILON2 = EPSILON * EPSILON;

    class Gravity {
        public:
            /**
             * @brief Apply gravitational forces to the bodies in the NewtonianState.
             *
             * @param state The NewtonianState containing the bodies and their properties.
             * @param dt The time step for the simulation.
             * @param mode The gravity computation mode (BruteForce or BarnesHut).
             * @param tree The octree containing the spatial partitioning.
             */
            static void apply(NewtonianState& state, double dt, GravityMode mode, Octree& tree);

            /**
             * @brief Compute the scalar mass from a given mass component.
             *
             * @param mass The mass component.
             * @return The scalar mass.
             */
            static components::ScalarMass computeScalarMass(const common::components::Mass& mass);

        private:
            using simd_t = physics::stdx::native_simd<double>;
            static constexpr std::size_t LANES = physics::SIMD_WIDTH; // Number of lanes in the SIMD vector

            struct MassCenters {
                    std::vector<double> mass;
                    std::vector<double> x;
                    std::vector<double> y;
                    std::vector<double> z;
            };

            static constexpr double BARNES_HUT_THETA = 0.5; // Threshold for Barnes-Hut approximation
            static constexpr double BARNES_HUT_THETA2 = BARNES_HUT_THETA * BARNES_HUT_THETA;

            /**
             * @brief Compute gravitational forces using the brute-force method.
             *
             * @param state The NewtonianState containing the bodies and their properties.
             */
            static void _computeGravity(NewtonianState& state);
            /**
             * @brief Compute gravitational forces using the Barnes-Hut algorithm.
             *
             * @param state The NewtonianState containing the bodies and their properties.
             * @param tree The octree containing the spatial partitioning.
             */
            static void _computeBarnesHutGravity(NewtonianState& state, Octree& tree);
            /**
             * @brief Compute the mass centers for each node in the octree.
             *
             * @param state The NewtonianState containing the bodies and their properties.
             * @param tree The octree containing the spatial partitioning.
             * @return The mass centers for each node.
             */
            static MassCenters _computeMassCenters(const NewtonianState& state, const Octree& tree);
            /**
             * @brief Add the gravitational force contribution from a source body to a target body.
             *
             * @param sourceMass The mass of the source body.
             * @param sourceX The x-coordinate of the source body.
             * @param sourceY The y-coordinate of the source body.
             * @param sourceZ The z-coordinate of the source body.
             * @param targetMass The mass of the target body.
             * @param targetX The x-coordinate of the target body.
             * @param targetY The y-coordinate of the target body.
             * @param targetZ The z-coordinate of the target body.
             * @param forceX The x-component of the gravitational force.
             * @param forceY The y-component of the gravitational force.
             * @param forceZ The z-component of the gravitational force.
             */
            static void _addForce(double sourceMass, double sourceX, double sourceY, double sourceZ,
                                  double targetMass, double targetX, double targetY, double targetZ,
                                  double& forceX, double& forceY, double& forceZ);
            /**
             * @brief Compute the gravitational force on a single body using the octree and mass centers.
             *
             * @param state The NewtonianState containing the bodies and their properties.
             * @param tree The octree containing the spatial partitioning.
             * @param centers The mass centers for each node.
             * @param slots The slots containing the body indices.
             * @param body The index of the body for which to compute the force.
             */
            static void _computeBodyForce(NewtonianState& state, const Octree& tree, const MassCenters& centers,
                                          const std::vector<std::uint32_t>& slots, std::uint32_t body);

            /**
             * @brief Compute the inverse distance between two points.
             *
             * @param disp The displacement vector between the two points.
             * @return The inverse distance.
             */
            static components::InverseDistance computeInverseDistance(const components::Displacement& disp);
    };
} // namespace physics::forces
