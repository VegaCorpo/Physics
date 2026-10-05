/**
 * White-box tests of physics::Collider.
 *
 * Every test builds a NewtonianState, an Octree, runs checkCollisions() and
 * compares the pairs with an O(n^2) brute force using the same contact rule
 * (center distance <= r1 + r2). Scenes are chosen so that the expected number
 * of contacts is known and non-trivial: the benchmark scenes of the engine
 * (bodies ~1e7 km apart, radii <= 1e5 km) contain no contact at all, see
 * SparseBenchmarkLikeCluster.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>
#include "Fixture.hpp"

using unit::Body;
using unit::Pair;
using unit::PairSet;

namespace {

    std::string describe(const std::vector<Pair>& pairs, const physics::NewtonianState& s, std::size_t limit = 10)
    {
        std::ostringstream out;
        std::size_t shown = 0;
        for (const auto& [i, j] : pairs) {
            if (shown == limit) {
                out << "  ... (" << pairs.size() - limit << " more)\n";
                break;
            }
            shown += 1;
            const double dx = s.posX[i] - s.posX[j];
            const double dy = s.posY[i] - s.posY[j];
            const double dz = s.posZ[i] - s.posZ[j];
            out << "  (" << i << ", " << j << ") dist=" << std::sqrt(dx * dx + dy * dy + dz * dz)
                << " r1+r2=" << s.radius[i] + s.radius[j] << "\n";
        }
        return out.str();
    }

    /// Full comparison with the brute force, with a readable diff on failure.
    void expectMatchesBruteForce(const std::vector<Body>& bodies, bool requireSomeContact = true)
    {
        const physics::NewtonianState state = unit::stateOf(bodies);
        const PairSet expected = unit::bruteForce(state);
        const unit::Detection d = unit::detect(state);

        if (requireSomeContact) {
            ASSERT_FALSE(expected.empty()) << "scene has no contact: the test would be vacuous";
        }

        EXPECT_EQ(d.selfPairs, 0u) << "collider reported a body colliding with itself";
        EXPECT_EQ(d.outOfRange, 0u) << "collider reported an index >= body count (padding slot?)";
        EXPECT_EQ(d.duplicates, 0u) << "collider reported the same pair more than once";

        const std::vector<Pair> missed = unit::missing(expected, d.pairs);
        const std::vector<Pair> spurious = unit::missing(d.pairs, expected);
        EXPECT_TRUE(missed.empty()) << missed.size() << " of " << expected.size()
                                    << " contacts were not detected:\n" << describe(missed, state);
        EXPECT_TRUE(spurious.empty()) << spurious.size() << " reported pairs are not in contact:\n"
                                      << describe(spurious, state);
    }

    /// Distance between two bodies in a set of 'anchor' bodies that pins the octree root.
    /// Eight anchors on the corners of a cube centred on the origin make the root node
    /// centred on the origin, so the planes x=0, y=0, z=0 are the first split planes.
    std::vector<Body> cornerAnchors(double extent, double radius = 1.0)
    {
        std::vector<Body> anchors;
        for (int corner = 0; corner < 8; corner += 1)
            anchors.push_back({(corner & 1) ? extent : -extent, (corner & 2) ? extent : -extent,
                               (corner & 4) ? extent : -extent, radius});
        return anchors;
    }

} // namespace

// --------------------------------------------------------------------------- trivial scenes

TEST(Collider, EmptyStateReportsNothing)
{
    const unit::Detection d = unit::detect(std::vector<Body>{});
    EXPECT_TRUE(d.raw.empty());
}

TEST(Collider, SingleBodyReportsNothing)
{
    const unit::Detection d = unit::detect({{0.0, 0.0, 0.0, 1000.0}});
    EXPECT_TRUE(d.raw.empty());
}

TEST(Collider, TwoOverlappingSpheresAreDetected)
{
    const unit::Detection d = unit::detect({{0.0, 0.0, 0.0, 1000.0}, {1500.0, 0.0, 0.0, 1000.0}});
    ASSERT_EQ(d.raw.size(), 1u);
    EXPECT_EQ(d.pairs, (PairSet{{0, 1}}));
}

TEST(Collider, TwoSpheresExactlyTouchingAreDetected)
{
    // dist == r1 + r2, all values exactly representable.
    const unit::Detection d = unit::detect({{0.0, 0.0, 0.0, 1000.0}, {2000.0, 0.0, 0.0, 1000.0}});
    EXPECT_EQ(d.pairs, (PairSet{{0, 1}}));
}

TEST(Collider, TwoSeparatedSpheresAreNotDetected)
{
    const unit::Detection d = unit::detect({{0.0, 0.0, 0.0, 1000.0}, {2000.5, 0.0, 0.0, 1000.0}});
    EXPECT_TRUE(d.raw.empty());
}

TEST(Collider, OverlapIsDetectedOnEveryAxisAndDiagonal)
{
    const double r = 1000.0;
    const double d = 1500.0;
    const std::vector<Body> pairs[] = {
        {{0, 0, 0, r}, {d, 0, 0, r}},
        {{0, 0, 0, r}, {0, d, 0, r}},
        {{0, 0, 0, r}, {0, 0, d, r}},
        {{0, 0, 0, r}, {-d, 0, 0, r}},
        {{0, 0, 0, r}, {0, -d, 0, r}},
        {{0, 0, 0, r}, {0, 0, -d, r}},
        {{0, 0, 0, r}, {d / std::sqrt(3.0), d / std::sqrt(3.0), d / std::sqrt(3.0), r}},
    };
    for (const auto& scene : pairs)
        EXPECT_EQ(unit::detect(scene).pairs, (PairSet{{0, 1}})) << "body at " << scene[1].x << ", " << scene[1].y
                                                               << ", " << scene[1].z;
}

TEST(Collider, CoincidentZeroRadiusBodiesAreInContact)
{
    // Same rule as the module: dist (0) <= r1 + r2 (0). Mirrors the coincident_bodies QA scene.
    const unit::Detection d = unit::detect({{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {1.0e6, 0.0, 0.0, 0.0}});
    EXPECT_EQ(d.pairs, (PairSet{{0, 1}}));
}

TEST(Collider, ChainOfThreeReportsOnlyAdjacentPairs)
{
    // A-B and B-C overlap, A-C do not.
    const unit::Detection d = unit::detect({{0.0, 0.0, 0.0, 1000.0}, {1500.0, 0.0, 0.0, 1000.0},
                                            {3000.0, 0.0, 0.0, 1000.0}});
    EXPECT_EQ(d.pairs, (PairSet{{0, 1}, {1, 2}}));
}

TEST(Collider, AsymmetricRadiiUseTheSumOfBothRadii)
{
    // r1 = 10, r2 = 990, dist = 1000 -> contact; dist = 1001 -> none.
    EXPECT_EQ(unit::detect({{0.0, 0.0, 0.0, 10.0}, {1000.0, 0.0, 0.0, 990.0}}).pairs, (PairSet{{0, 1}}));
    EXPECT_TRUE(unit::detect({{0.0, 0.0, 0.0, 10.0}, {1001.0, 0.0, 0.0, 990.0}}).raw.empty());
    // Regression: the second radius used to be read from posZ instead of radius.
    EXPECT_EQ(unit::detect({{0.0, 0.0, 0.0, 10.0}, {1000.0, 0.0, 5000.0, 990.0}}).raw.size(), 0u);
    EXPECT_EQ(unit::detect({{0.0, 0.0, 0.0, 10.0}, {0.0, 0.0, 1000.0, 990.0}}).pairs, (PairSet{{0, 1}}));
}

// --------------------------------------------------------------------------- octree interaction

TEST(Collider, OverlapWithinOneLeafIsDetectedWhenTreeIsSubdivided)
{
    // 8 corner anchors force the root to subdivide (> LEAF_CAPACITY bodies).
    // Both overlapping bodies sit deep inside the same octant.
    std::vector<Body> bodies = cornerAnchors(1.0e6);
    bodies.push_back({2.0e5, 2.0e5, 2.0e5, 500.0});
    bodies.push_back({2.0e5 + 600.0, 2.0e5, 2.0e5, 500.0});

    physics::Octree octree;
    const physics::NewtonianState state = unit::stateOf(bodies);
    octree.build(state);
    ASSERT_GT(octree.nodes().size(), 1u) << "precondition: the tree must be subdivided";
    ASSERT_EQ(unit::leafOf(octree, 8), unit::leafOf(octree, 9)) << "precondition: same leaf";

    EXPECT_EQ(unit::detect(state).pairs, (PairSet{{8, 9}}));
}

TEST(Collider, OverlapAcrossOctreeLeafBoundaryIsDetected)
{
    // Two bodies overlapping each other but on opposite sides of the root split
    // plane x = 0: the octree puts them in different leaves. A leaf-local pair
    // check misses this contact.
    std::vector<Body> bodies = cornerAnchors(1.0e6);
    bodies.push_back({-100.0, 0.0, 0.0, 500.0});
    bodies.push_back({+100.0, 0.0, 0.0, 500.0});

    physics::Octree octree;
    const physics::NewtonianState state = unit::stateOf(bodies);
    octree.build(state);
    ASSERT_GT(octree.nodes().size(), 1u) << "precondition: the tree must be subdivided";
    ASSERT_NE(unit::leafOf(octree, 8), unit::leafOf(octree, 9)) << "precondition: different leaves";

    EXPECT_EQ(unit::detect(state).pairs, (PairSet{{8, 9}}));
}

TEST(Collider, OverlapAcrossEverySplitPlaneIsDetected)
{
    // One overlapping pair straddling each of the three root split planes, and
    // one straddling all three at once (bodies in diagonally opposite octants).
    std::vector<Body> bodies = cornerAnchors(1.0e6);
    const double r = 500.0;
    const double off = 100.0;
    // x plane, far from the others
    bodies.push_back({-off, 3.0e5, 3.0e5, r});
    bodies.push_back({+off, 3.0e5, 3.0e5, r});
    // y plane
    bodies.push_back({3.0e5, -off, -3.0e5, r});
    bodies.push_back({3.0e5, +off, -3.0e5, r});
    // z plane
    bodies.push_back({-3.0e5, -3.0e5, -off, r});
    bodies.push_back({-3.0e5, -3.0e5, +off, r});
    // all three
    bodies.push_back({-off, -off, -off, r});
    bodies.push_back({+off, +off, +off, r});

    expectMatchesBruteForce(bodies);
}

TEST(Collider, LargeBodyOverlappingManySmallOnesAcrossLeaves)
{
    // A big body at the origin covers the whole cluster of 200 small bodies which
    // are spread over many leaves: all 200 pairs (big, small) must be reported.
    std::vector<Body> bodies = unit::randomCluster(200, 42, 5.0e5, 1.0, 1.0);
    bodies.insert(bodies.begin(), Body{0.0, 0.0, 0.0, 1.0e6}); // covers radius sqrt(3)*5e5 ~ 8.7e5

    const physics::NewtonianState state = unit::stateOf(bodies);
    const unit::Detection d = unit::detect(state);
    std::size_t withBig = 0;
    for (const auto& [a, b] : d.pairs)
        withBig += (a == 0);
    EXPECT_EQ(withBig, 200u) << "every small body overlaps the big one";
    expectMatchesBruteForce(bodies);
}

TEST(Collider, ManyCoincidentBodiesHitTheDepthLimit)
{
    // 20 bodies at the same point cannot be separated by subdivision: the octree
    // stops at MAX_DEPTH with a leaf holding all of them -> 190 pairs.
    std::vector<Body> bodies(20, Body{1.0e3, -2.0e3, 3.0e3, 1.0});
    bodies.push_back({1.0e9, 1.0e9, 1.0e9, 1.0}); // far away so the root has a non-zero extent

    const unit::Detection d = unit::detect(bodies);
    EXPECT_EQ(d.pairs.size(), 190u);
    EXPECT_EQ(d.duplicates, 0u);
    EXPECT_EQ(d.selfPairs, 0u);
}

// --------------------------------------------------------------------------- random scenes vs brute force

TEST(Collider, DenseRandomClusterMatchesBruteForce)
{
    // 1000 bodies in a 200 000 km cube with radii 1000-5000 km: hundreds of contacts.
    expectMatchesBruteForce(unit::randomCluster(1000, 1, 1.0e5, 1.0e3, 5.0e3));
}

TEST(Collider, DenseRandomClusterMatchesBruteForceOtherSeeds)
{
    for (unsigned seed = 2; seed < 6; seed += 1)
        expectMatchesBruteForce(unit::randomCluster(500, seed, 5.0e4, 5.0e2, 4.0e3));
}

TEST(Collider, MixedRadiiClusterMatchesBruteForce)
{
    // A few huge bodies among many tiny ones: contacts span leaves at very
    // different depths.
    std::vector<Body> bodies = unit::randomCluster(800, 7, 1.0e6, 10.0, 100.0);
    for (const Body& big : unit::randomCluster(5, 8, 1.0e6, 2.0e5, 4.0e5))
        bodies.push_back(big);
    expectMatchesBruteForce(bodies);
}

TEST(Collider, TwoDistantDenseClustersMatchBruteForce)
{
    // Two dense clusters 2e9 km apart: the tree is very deep around each one.
    std::vector<Body> bodies = unit::randomCluster(300, 11, 2.0e4, 1.0e3, 3.0e3);
    for (Body b : unit::randomCluster(300, 12, 2.0e4, 1.0e3, 3.0e3)) {
        b.x += 2.0e9;
        b.y -= 1.0e9;
        bodies.push_back(b);
    }
    expectMatchesBruteForce(bodies);
}

TEST(Collider, SparseBenchmarkLikeClusterHasNoContactByConstruction)
{
    // Same statistics as scenes/benchmark_1000.json (generator.py): 1000 bodies
    // in a 2e9 km cube, radii <= 1e5 km. Bodies are ~1e7 km apart, so no pair
    // is in contact: zero collisions is the correct answer for those scenes.
    const std::vector<Body> bodies = unit::randomCluster(1000, 2024, 1.0e9, 0.0, 1.0e5);
    const physics::NewtonianState state = unit::stateOf(bodies);
    ASSERT_TRUE(unit::bruteForce(state).empty());
    EXPECT_TRUE(unit::detect(state).raw.empty());
}

// --------------------------------------------------------------------------- robustness

TEST(Collider, PaddingSlotsAreNeverReportedAsBodies)
{
    // NewtonianState pads its arrays to a multiple of BLOCK with zeroed slots
    // (position 0, radius 0). A real body at the origin must not "collide" with
    // those slots, for any count around the block boundary.
    for (std::size_t n : {1u, 2u, 7u, 8u, 9u, 15u, 16u, 17u, 31u, 32u, 33u}) {
        std::vector<Body> bodies = unit::randomCluster(n - 1, static_cast<unsigned>(n), 1.0e6, 1.0, 1.0);
        bodies.insert(bodies.begin(), Body{0.0, 0.0, 0.0, 10.0});
        const physics::NewtonianState state = unit::stateOf(bodies);
        ASSERT_GE(state.paddedSize(), state.size());
        const unit::Detection d = unit::detect(state);
        EXPECT_EQ(d.outOfRange, 0u) << "n=" << n;
        EXPECT_EQ(d.pairs, unit::bruteForce(state)) << "n=" << n;
    }
}

TEST(Collider, ResultIsDeterministic)
{
    const std::vector<Body> bodies = unit::randomCluster(600, 99, 1.0e5, 1.0e3, 5.0e3);
    const unit::Detection a = unit::detect(bodies);
    const unit::Detection b = unit::detect(bodies);
    EXPECT_EQ(a.raw, b.raw);
}

TEST(Collider, ResultIsIndependentOfBodyOrder)
{
    std::vector<Body> bodies = unit::randomCluster(600, 5, 1.0e5, 1.0e3, 5.0e3);
    const PairSet reference = unit::detect(bodies).pairs;

    std::vector<std::uint32_t> order(bodies.size());
    for (std::uint32_t i = 0; i < order.size(); i += 1)
        order[i] = i;
    std::mt19937 rng(77);
    std::shuffle(order.begin(), order.end(), rng);

    std::vector<Body> shuffled;
    for (std::uint32_t idx : order)
        shuffled.push_back(bodies[idx]);
    PairSet mapped;
    for (const auto& [a, b] : unit::detect(shuffled).pairs)
        mapped.insert(unit::ordered(order[a], order[b]));
    EXPECT_EQ(mapped, reference);
}

TEST(Collider, ResultIsIndependentOfTranslation)
{
    std::vector<Body> bodies = unit::randomCluster(600, 6, 1.0e5, 1.0e3, 5.0e3);
    const PairSet reference = unit::detect(bodies).pairs;
    for (Body& b : bodies) {
        b.x += 3.0e8;
        b.y -= 7.0e8;
        b.z += 1.0e8;
    }
    EXPECT_EQ(unit::detect(bodies).pairs, reference);
}

TEST(Collider, ReusingTheSameColliderAndOctreeAcrossFramesGivesFreshResults)
{
    // The engine keeps one Octree and one Collider and rebuilds every frame:
    // results must not accumulate or leak from a previous frame.
    physics::Octree octree;
    physics::Collider collider;

    physics::NewtonianState touching = unit::stateOf({{0.0, 0.0, 0.0, 1000.0}, {1500.0, 0.0, 0.0, 1000.0}});
    octree.build(touching);
    EXPECT_EQ(collider.checkCollisions(touching, octree).size(), 1u);

    physics::NewtonianState apart = unit::stateOf({{0.0, 0.0, 0.0, 1000.0}, {1.0e6, 0.0, 0.0, 1000.0}});
    octree.build(apart);
    EXPECT_EQ(collider.checkCollisions(apart, octree).size(), 0u);

    octree.build(touching);
    EXPECT_EQ(collider.checkCollisions(touching, octree).size(), 1u);
}
