#pragma once

#include <array>
#include <cassert>
#include <cstddef>
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

            static constexpr double BARNES_HUT_THETA = 0.5; // Threshold for Barnes-Hut approximation
            static constexpr double BARNES_HUT_THETA2 = BARNES_HUT_THETA * BARNES_HUT_THETA;

            // Each expansion pops one node and pushes up to 8 children, so a path of
            // depth D holds at most 7 * D + 1 pending nodes.
            static constexpr std::size_t MAX_TREE_DEPTH = 32;
            static constexpr std::size_t STACK_CAPACITY = 7 * MAX_TREE_DEPTH + 1;

            struct Vec3 {
                    double x {};
                    double y {};
                    double z {};

                    Vec3& operator+=(const Vec3& other)
                    {
                        x += other.x;
                        y += other.y;
                        z += other.z;
                        return *this;
                    }

                    friend Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
                    friend Vec3 operator*(const Vec3& v, double scalar) { return {v.x * scalar, v.y * scalar, v.z * scalar}; }
                    friend Vec3 operator/(const Vec3& v, double scalar) { return {v.x / scalar, v.y / scalar, v.z / scalar}; }

                    double norm2() const { return x * x + y * y + z * z; }
            };

            struct PointMass {
                    double mass {};
                    Vec3 pos {};
            };

            // Sum of mass and mass-weighted positions, before division by the total mass.
            struct Moment {
                    double mass {};
                    Vec3 weighted {};

                    Moment& operator+=(const PointMass& point)
                    {
                        mass += point.mass;
                        weighted += point.pos * point.mass;
                        return *this;
                    }
            };

            struct MassCenters {
                    std::vector<double> mass;
                    std::vector<double> x;
                    std::vector<double> y;
                    std::vector<double> z;

                    explicit MassCenters(std::size_t size) : mass(size), x(size), y(size), z(size) {}

                    PointMass at(std::size_t index) const { return {mass[index], {x[index], y[index], z[index]}}; }

                    void set(std::size_t index, const Moment& moment)
                    {
                        mass[index] = moment.mass;
                        if (moment.mass == 0.0)
                            return;
                        const Vec3 center = moment.weighted / moment.mass;
                        x[index] = center.x;
                        y[index] = center.y;
                        z[index] = center.z;
                    }
            };

            // Fixed-size stack living on the thread stack: no heap allocation per body.
            struct NodeStack {
                    std::array<std::uint32_t, STACK_CAPACITY> data;
                    std::size_t size = 0;

                    bool empty() const { return size == 0; }

                    void push(std::uint32_t node)
                    {
                        assert(size < data.size());
                        data[size++] = node;
                    }

                    std::uint32_t pop() { return data[--size]; }
            };

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
             * @brief Compute the inverse of the octree permutation (body index -> slot).
             *
             * @param permutations The octree permutations (slot -> body index).
             * @param bodyCount The number of bodies.
             * @return The slot of each body.
             */
            static std::vector<std::uint32_t> _computeSlots(const std::vector<std::uint32_t>& permutations,
                                                            std::size_t bodyCount);
            /**
             * @brief Compute the mass centers for each node in the octree.
             *
             * @param state The NewtonianState containing the bodies and their properties.
             * @param tree The octree containing the spatial partitioning.
             * @return The mass centers for each node.
             */
            static MassCenters _computeMassCenters(const NewtonianState& state, const Octree& tree);
            /**
             * @brief Accumulate the mass and weighted positions of the bodies held by a leaf.
             */
            static Moment _leafMoment(const physics::Node& node, const NewtonianState& state,
                                      const std::vector<std::uint32_t>& permutations);
            /**
             * @brief Accumulate the mass and weighted positions of the children of an internal node.
             */
            static Moment _internalMoment(const physics::Node& node, const MassCenters& centers);
            /**
             * @brief Build the point mass of a body.
             */
            static PointMass _bodyPoint(const NewtonianState& state, std::uint32_t body);
            /**
             * @brief Compute the gravitational force exerted by a source on a target.
             *
             * @param source The source point mass.
             * @param target The target point mass.
             * @return The force applied on the target.
             */
            static Vec3 _gravityFrom(const PointMass& source, const PointMass& target);
            /**
             * @brief Compute the gravitational force on a single body using the octree and mass centers.
             *
             * @param state The NewtonianState containing the bodies and their properties.
             * @param tree The octree containing the spatial partitioning.
             * @param centers The mass centers for each node.
             * @param slots The slot of each body in the octree permutation.
             * @param body The index of the body for which to compute the force.
             */
            static void _computeBodyForce(NewtonianState& state, const Octree& tree, const MassCenters& centers,
                                          const std::vector<std::uint32_t>& slots, std::uint32_t body);
            /**
             * @brief Compute the force applied on the target by every other body of a leaf.
             */
            static Vec3 _leafForce(const physics::Node& node, const NewtonianState& state,
                                   const std::vector<std::uint32_t>& permutations, std::uint32_t body,
                                   const PointMass& target);
            /**
             * @brief Tell whether a node is far enough to be approximated by its mass center (theta criterion).
             */
            static bool _canApproximate(const physics::Node& node, const Vec3& center, const Vec3& target);
            /**
             * @brief Push the non-empty children of a node on the traversal stack.
             */
            static void _pushChildren(const Octree& tree, const physics::Node& node, NodeStack& stack);
            /**
             * @brief Tell whether a node has no children.
             */
            static bool _isLeaf(const physics::Node& node);
            /**
             * @brief Tell whether a slot belongs to the range covered by a node.
             */
            static bool _containsSlot(const physics::Node& node, std::uint32_t slot);

            /**
             * @brief Compute the inverse distance between two points.
             *
             * @param disp The displacement vector between the two points.
             * @return The inverse distance.
             */
            static components::InverseDistance computeInverseDistance(const components::Displacement& disp);
    };
} // namespace physics::forces
