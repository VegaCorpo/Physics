#pragma once

/**
 * Shared helpers for the white-box collision tests.
 *
 * A test describes bodies as {x, y, z, radius}, turns them into a
 * physics::NewtonianState through the same syncIn() path the engine uses (so
 * the float radius round trip is identical), builds the Octree and runs the
 * Collider. The result is compared with an O(n^2) brute force that applies the
 * exact same contact criterion (center distance <= r1 + r2).
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <random>
#include <set>
#include <utility>
#include <vector>
#include <types/World.hpp>
#include "collisions/collider.hpp"
#include "components/NewtonianState.hpp"
#include "spatial/Octree.hpp"

namespace unit {

    struct Body {
            double x;
            double y;
            double z;
            double radius;
    };

    using Pair = std::pair<std::uint32_t, std::uint32_t>;
    using PairSet = std::set<Pair>;

    inline common::WorldState worldOf(const std::vector<Body>& bodies)
    {
        common::WorldState world;
        for (std::size_t i = 0; i < bodies.size(); i += 1) {
            world.entities.push_back(i);
            world.positions.push_back({bodies[i].x, bodies[i].y, bodies[i].z});
            world.velocities.push_back({0.0, 0.0, 0.0});
            world.accelerations.push_back({0.0, 0.0, 0.0});
            world.mass.push_back({1.0f, 24});
            world.radius.push_back({static_cast<float>(bodies[i].radius)});
        }
        return world;
    }

    inline physics::NewtonianState stateOf(const std::vector<Body>& bodies)
    {
        physics::NewtonianState state;
        state.syncIn(worldOf(bodies));
        return state;
    }

    inline Pair ordered(std::uint32_t a, std::uint32_t b)
    {
        return a < b ? Pair{a, b} : Pair{b, a};
    }

    /// Same criterion as the module: contact when center distance <= r1 + r2.
    inline bool overlaps(const physics::NewtonianState& s, std::uint32_t i, std::uint32_t j)
    {
        const double dx = s.posX[i] - s.posX[j];
        const double dy = s.posY[i] - s.posY[j];
        const double dz = s.posZ[i] - s.posZ[j];
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        return dist <= s.radius[i] + s.radius[j];
    }

    inline PairSet bruteForce(const physics::NewtonianState& s)
    {
        PairSet out;
        const auto n = static_cast<std::uint32_t>(s.size());
        for (std::uint32_t i = 0; i < n; i += 1)
            for (std::uint32_t j = i + 1; j < n; j += 1)
                if (overlaps(s, i, j))
                    out.insert({i, j});
        return out;
    }

    struct Detection {
            std::vector<Pair> raw;    ///< exactly what the collider returned
            PairSet pairs;            ///< normalised (min, max), duplicates removed
            std::size_t selfPairs = 0;
            std::size_t outOfRange = 0;
            std::size_t duplicates = 0;
    };

    inline Detection detect(const physics::NewtonianState& state)
    {
        physics::Octree octree;
        physics::Collider collider;
        physics::NewtonianState mutableState = state;

        octree.build(mutableState);
        Detection d;
        d.raw = collider.check_collisions(mutableState, octree);
        for (const auto& [a, b] : d.raw) {
            if (a == b)
                d.selfPairs += 1;
            if (a >= state.size() || b >= state.size())
                d.outOfRange += 1;
            if (!d.pairs.insert(ordered(a, b)).second)
                d.duplicates += 1;
        }
        return d;
    }

    inline Detection detect(const std::vector<Body>& bodies)
    {
        return detect(stateOf(bodies));
    }

    /// Bodies uniformly distributed in a cube of half-side `extent`, radii in [rMin, rMax].
    inline std::vector<Body> randomCluster(std::size_t n, unsigned seed, double extent, double rMin, double rMax)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> pos(-extent, extent);
        std::uniform_real_distribution<double> rad(rMin, rMax);
        std::vector<Body> bodies;
        bodies.reserve(n);
        for (std::size_t i = 0; i < n; i += 1)
            bodies.push_back({pos(rng), pos(rng), pos(rng), rad(rng)});
        return bodies;
    }

    /// Index of the leaf node that holds `body`, or INVALID_NODE when not found.
    inline std::uint32_t leafOf(const physics::Octree& octree, std::uint32_t body)
    {
        const auto& nodes = octree.nodes();
        const auto& perm = octree.permutations();
        for (std::uint32_t n = 0; n < nodes.size(); n += 1) {
            if (nodes[n].first_child != physics::INVALID_NODE)
                continue;
            for (std::uint32_t slot = nodes[n].begin; slot < nodes[n].begin + nodes[n].count; slot += 1)
                if (perm[slot] == body)
                    return n;
        }
        return physics::INVALID_NODE;
    }

    inline std::vector<Pair> missing(const PairSet& expected, const PairSet& actual)
    {
        std::vector<Pair> out;
        std::set_difference(expected.begin(), expected.end(), actual.begin(), actual.end(),
                            std::back_inserter(out));
        return out;
    }

} // namespace unit
