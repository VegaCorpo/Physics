/**
 * Structural invariants of physics::Octree that the collider relies on.
 */

#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <vector>
#include "Fixture.hpp"

using unit::Body;

namespace {

    struct Walk {
            std::size_t leaves = 0;
            std::size_t bodiesInLeaves = 0;
            std::size_t overfullLeaves = 0;
            std::size_t bodiesOutsideLeaf = 0;
            std::size_t badChildRanges = 0;
    };

    void walk(const physics::Octree& octree, const physics::NewtonianState& state, std::uint32_t index, Walk& w)
    {
        const auto& nodes = octree.nodes();
        const physics::Node& node = nodes[index];

        if (node.firstChild != physics::INVALID_NODE) {
            std::uint32_t next = node.begin;
            std::uint32_t total = 0;
            for (std::uint32_t o = 0; o < physics::OCTANT_COUNT; o += 1) {
                const physics::Node& child = nodes[node.firstChild + o];
                if (child.begin != next)
                    w.badChildRanges += 1;
                next = child.begin + child.count;
                total += child.count;
                walk(octree, state, node.firstChild + o, w);
            }
            if (total != node.count)
                w.badChildRanges += 1;
            return;
        }

        w.leaves += 1;
        w.bodiesInLeaves += node.count;
        if (node.count > physics::LEAF_CAPACITY && node.depth < physics::MAX_DEPTH)
            w.overfullLeaves += 1;
        const double tol = node.halfSize * 1e-9;
        for (std::uint32_t slot = node.begin; slot < node.begin + node.count; slot += 1) {
            const std::uint32_t body = octree.permutations()[slot];
            const bool inside = std::abs(state.posX[body] - node.centerX) <= node.halfSize + tol &&
                                std::abs(state.posY[body] - node.centerY) <= node.halfSize + tol &&
                                std::abs(state.posZ[body] - node.centerZ) <= node.halfSize + tol;
            if (!inside)
                w.bodiesOutsideLeaf += 1;
        }
    }

    Walk check(const std::vector<Body>& bodies)
    {
        const physics::NewtonianState state = unit::stateOf(bodies);
        physics::Octree octree;
        octree.build(state);

        Walk w;
        if (!octree.nodes().empty())
            walk(octree, state, physics::FIRST_NODE, w);

        std::vector<std::uint32_t> perm = octree.permutations();
        std::sort(perm.begin(), perm.end());
        for (std::uint32_t i = 0; i < perm.size(); i += 1)
            EXPECT_EQ(perm[i], i) << "permutations() is not a permutation of 0..n-1";
        EXPECT_EQ(perm.size(), bodies.size());
        return w;
    }

} // namespace

TEST(Octree, EmptyStateBuildsNoNode)
{
    physics::Octree octree;
    octree.build(unit::stateOf({}));
    EXPECT_TRUE(octree.nodes().empty());
    EXPECT_TRUE(octree.permutations().empty());
}

TEST(Octree, SmallSceneIsASingleLeaf)
{
    const physics::NewtonianState state = unit::stateOf(unit::randomCluster(physics::LEAF_CAPACITY, 3, 1e6, 1, 1));
    physics::Octree octree;
    octree.build(state);
    ASSERT_EQ(octree.nodes().size(), 1u);
    EXPECT_EQ(octree.nodes()[0].count, physics::LEAF_CAPACITY);
    EXPECT_EQ(octree.nodes()[0].firstChild, physics::INVALID_NODE);
}

TEST(Octree, LeavesPartitionTheBodiesAndContainThem)
{
    const std::vector<Body> bodies = unit::randomCluster(2000, 21, 1.0e9, 1.0, 1.0);
    const Walk w = check(bodies);
    EXPECT_EQ(w.bodiesInLeaves, bodies.size());
    EXPECT_EQ(w.bodiesOutsideLeaf, 0u);
    EXPECT_EQ(w.overfullLeaves, 0u);
    EXPECT_EQ(w.badChildRanges, 0u);
    EXPECT_GT(w.leaves, 1u);
}

TEST(Octree, RootBoxContainsEveryBody)
{
    const std::vector<Body> bodies = unit::randomCluster(500, 22, 1.0e9, 1.0, 1.0);
    const physics::NewtonianState state = unit::stateOf(bodies);
    physics::Octree octree;
    octree.build(state);
    const physics::Node& root = octree.nodes()[physics::FIRST_NODE];
    for (std::uint32_t i = 0; i < state.size(); i += 1) {
        EXPECT_LE(std::abs(state.posX[i] - root.centerX), root.halfSize);
        EXPECT_LE(std::abs(state.posY[i] - root.centerY), root.halfSize);
        EXPECT_LE(std::abs(state.posZ[i] - root.centerZ), root.halfSize);
    }
}

TEST(Octree, CoincidentBodiesStopAtDepthLimit)
{
    std::vector<Body> bodies(physics::LEAF_CAPACITY + 1, Body{0.0, 0.0, 0.0, 1.0});
    bodies.push_back({1.0e6, 0.0, 0.0, 1.0});
    const physics::NewtonianState state = unit::stateOf(bodies);
    physics::Octree octree;
    octree.build(state);
    std::uint32_t deepest = 0;
    for (const physics::Node& n : octree.nodes())
        deepest = std::max(deepest, n.depth);
    EXPECT_EQ(deepest, physics::MAX_DEPTH);
    const Walk w = check(bodies);
    EXPECT_EQ(w.bodiesInLeaves, bodies.size());
}

TEST(Octree, RebuildClearsPreviousTree)
{
    physics::Octree octree;
    octree.build(unit::stateOf(unit::randomCluster(2000, 23, 1.0e9, 1.0, 1.0)));
    const std::size_t big = octree.nodes().size();
    octree.build(unit::stateOf(unit::randomCluster(3, 24, 1.0e9, 1.0, 1.0)));
    EXPECT_EQ(octree.nodes().size(), 1u);
    EXPECT_EQ(octree.permutations().size(), 3u);
    EXPECT_GT(big, 1u);
}
