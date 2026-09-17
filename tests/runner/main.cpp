/**
 * physics_qa_runner: black-box driver for modules implementing common::IPhysicsEngine.
 *
 * Modes
 *   simulate  --lib L --scene S --dt D --frames F [--sample-every K] [--snapshots]
 *             [--reverse-at R] [--G g] --out O
 *             Drives the library exactly like Core does (syncIn, update, syncOut per
 *             frame) and records conserved quantities and, optionally, full state
 *             snapshots at sample points.
 *   reference --scene S --dt D --frames F [--sample-every K] [--substeps N]
 *             [--softening e] [--G g] --out O
 *             Same output layout, produced by the built-in long double RK4 integrator.
 *   bench     --lib L --scene S --dt D --frames F [--warmup W] [--repeats R] --out O
 *             Times update() and the whole frame contract, fresh engine per repeat.
 *
 * All numbers are written with round-trip precision so consumers can compare bitwise.
 */

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <nlohmann/json.hpp>
#include "Invariants.hpp"
#include "Plugin.hpp"
#include "Reference.hpp"
#include "Scene.hpp"

namespace {

    constexpr const char* RUNNER_VERSION = "1.0.0";
    constexpr double DEFAULT_G = 6.67430e-20; // km^3 kg^-1 s^-2, matches the module

    using json = nlohmann::json;
    using Clock = std::chrono::steady_clock;

    struct Options {
            std::string mode;
            std::string lib;
            std::string scene;
            std::string out;
            double dt = 7200.0;
            double G = DEFAULT_G;
            double softening = 0.0;
            std::size_t frames = 1;
            std::size_t sampleEvery = 0;
            std::size_t substeps = 16;
            std::size_t warmup = 0;
            std::size_t repeats = 1;
            long reverseAt = -1;
            bool snapshots = false;
            bool potential = true;
    };

    [[noreturn]] void usage(const std::string& error)
    {
        std::cerr << "error: " << error << "\n"
                  << "usage: physics_qa_runner <simulate|reference|bench> [options]\n"
                  << "  --lib PATH --scene PATH --out PATH --dt SECONDS --frames N\n"
                  << "  --sample-every N --snapshots --reverse-at FRAME --no-potential\n"
                  << "  --substeps N --softening KM --G VALUE --warmup N --repeats N\n";
        std::exit(2);
    }

    Options parse(int argc, char** argv)
    {
        if (argc < 2)
            usage("missing mode");
        Options o;
        o.mode = argv[1];

        auto need = [&](int& i) -> std::string
        {
            if (i + 1 >= argc)
                usage(std::string("missing value for ") + argv[i]);
            return argv[++i];
        };

        for (int i = 2; i < argc; i += 1) {
            const std::string a = argv[i];
            if (a == "--lib")
                o.lib = need(i);
            else if (a == "--scene")
                o.scene = need(i);
            else if (a == "--out")
                o.out = need(i);
            else if (a == "--dt")
                o.dt = std::stod(need(i));
            else if (a == "--G")
                o.G = std::stod(need(i));
            else if (a == "--softening")
                o.softening = std::stod(need(i));
            else if (a == "--frames")
                o.frames = std::stoul(need(i));
            else if (a == "--sample-every")
                o.sampleEvery = std::stoul(need(i));
            else if (a == "--substeps")
                o.substeps = std::stoul(need(i));
            else if (a == "--warmup")
                o.warmup = std::stoul(need(i));
            else if (a == "--repeats")
                o.repeats = std::stoul(need(i));
            else if (a == "--reverse-at")
                o.reverseAt = std::stol(need(i));
            else if (a == "--snapshots")
                o.snapshots = true;
            else if (a == "--no-potential")
                o.potential = false;
            else
                usage("unknown option " + a);
        }

        if (o.mode != "simulate" && o.mode != "reference" && o.mode != "bench")
            usage("unknown mode " + o.mode);
        if (o.scene.empty() || o.out.empty())
            usage("--scene and --out are required");
        if (o.mode != "reference" && o.lib.empty())
            usage("--lib is required");
        if (o.sampleEvery == 0)
            o.sampleEvery = o.frames == 0 ? 1 : o.frames;
        return o;
    }

    double millis(Clock::time_point a, Clock::time_point b)
    {
        return std::chrono::duration<double, std::milli>(b - a).count();
    }

    json sample(std::size_t frame, double dt, const common::WorldState& world, const Options& o)
    {
        const qa::Invariants inv = qa::computeInvariants(world, o.G, o.potential);
        json s;
        s["frame"] = frame;
        s["t"] = static_cast<double>(frame) * dt;
        s["kinetic"] = static_cast<double>(inv.kinetic);
        s["potential"] = static_cast<double>(inv.potential);
        s["energy"] = static_cast<double>(inv.kinetic + inv.potential);
        s["momentum"] = {static_cast<double>(inv.momentum[0]), static_cast<double>(inv.momentum[1]),
                         static_cast<double>(inv.momentum[2])};
        s["angular_momentum"] = {static_cast<double>(inv.angularMomentum[0]),
                                 static_cast<double>(inv.angularMomentum[1]),
                                 static_cast<double>(inv.angularMomentum[2])};
        s["center_of_mass"] = {static_cast<double>(inv.centerOfMass[0]), static_cast<double>(inv.centerOfMass[1]),
                               static_cast<double>(inv.centerOfMass[2])};
        s["non_finite"] = inv.nonFinite;

        if (o.snapshots) {
            json pos = json::array();
            json vel = json::array();
            for (std::size_t i = 0; i < world.positions.size(); i += 1) {
                pos.push_back({world.positions[i].x, world.positions[i].y, world.positions[i].z});
                vel.push_back({world.velocities[i].x, world.velocities[i].y, world.velocities[i].z});
            }
            s["positions"] = std::move(pos);
            s["velocities"] = std::move(vel);
        }
        return s;
    }

    json header(const Options& o, const common::WorldState& world)
    {
        json h;
        h["runner_version"] = RUNNER_VERSION;
        h["mode"] = o.mode;
        h["scene"] = o.scene;
        h["bodies"] = world.positions.size();
        h["dt"] = o.dt;
        h["frames"] = o.frames;
        h["G"] = o.G;
        h["hardware_threads"] = std::thread::hardware_concurrency();
        return h;
    }

    int runSimulate(const Options& o)
    {
        common::WorldState world = qa::loadScene(o.scene);
        json out = header(o, world);

        qa::Plugin plugin(o.lib);
        std::unique_ptr<common::IPhysicsEngine> engine = plugin.create();
        if (!engine)
            throw std::runtime_error("get_engine() returned null");
        out["engine"] = engine->getName();

        const auto t0 = Clock::now();
        engine->init(world);
        out["init_ms"] = millis(t0, Clock::now());

        json samples = json::array();
        std::vector<double> updateMs;
        updateMs.reserve(o.frames);
        samples.push_back(sample(0, o.dt, world, o));

        for (std::size_t frame = 1; frame <= o.frames; frame += 1) {
            engine->syncIn(world);
            const auto a = Clock::now();
            engine->update(o.dt);
            updateMs.push_back(millis(a, Clock::now()));
            world = engine->syncOut();

            if (o.reverseAt >= 0 && frame == static_cast<std::size_t>(o.reverseAt)) {
                for (auto& v : world.velocities) {
                    v.x = -v.x;
                    v.y = -v.y;
                    v.z = -v.z;
                }
            }
            if (frame % o.sampleEvery == 0 || frame == o.frames)
                samples.push_back(sample(frame, o.dt, world, o));
        }
        engine->shutdown();
        engine.reset();

        out["samples"] = std::move(samples);
        out["update_ms"] = std::move(updateMs);
        std::ofstream(o.out) << out.dump();
        return 0;
    }

    int runReference(const Options& o)
    {
        common::WorldState world = qa::loadScene(o.scene);
        json out = header(o, world);
        out["engine"] = "ReferenceRK4LongDouble";
        out["substeps"] = o.substeps;
        out["softening"] = o.softening;

        qa::Reference ref(world, o.G, static_cast<long double>(o.softening) * o.softening);
        json samples = json::array();
        samples.push_back(sample(0, o.dt, world, o));

        for (std::size_t frame = 1; frame <= o.frames; frame += 1) {
            ref.step(o.dt, o.substeps);
            if (frame % o.sampleEvery == 0 || frame == o.frames) {
                ref.writeTo(world);
                samples.push_back(sample(frame, o.dt, world, o));
            }
        }
        out["samples"] = std::move(samples);
        std::ofstream(o.out) << out.dump();
        return 0;
    }

    int runBench(const Options& o)
    {
        const common::WorldState initial = qa::loadScene(o.scene);
        json out = header(o, initial);
        out["warmup"] = o.warmup;
        out["repeats"] = o.repeats;

        qa::Plugin plugin(o.lib);
        json repeats = json::array();

        for (std::size_t r = 0; r < o.repeats; r += 1) {
            common::WorldState world = initial;
            std::unique_ptr<common::IPhysicsEngine> engine = plugin.create();
            if (!engine)
                throw std::runtime_error("get_engine() returned null");
            if (r == 0)
                out["engine"] = engine->getName();

            const auto t0 = Clock::now();
            engine->init(world);
            const double initMs = millis(t0, Clock::now());

            for (std::size_t w = 0; w < o.warmup; w += 1) {
                engine->syncIn(world);
                engine->update(o.dt);
                world = engine->syncOut();
            }

            std::vector<double> updateMs;
            std::vector<double> frameMs;
            updateMs.reserve(o.frames);
            frameMs.reserve(o.frames);
            for (std::size_t f = 0; f < o.frames; f += 1) {
                const auto a = Clock::now();
                engine->syncIn(world);
                const auto b = Clock::now();
                engine->update(o.dt);
                const auto c = Clock::now();
                world = engine->syncOut();
                const auto d = Clock::now();
                updateMs.push_back(millis(b, c));
                frameMs.push_back(millis(a, d));
            }
            engine->shutdown();
            engine.reset();

            const qa::Invariants inv = qa::computeInvariants(world, o.G, false);
            json rep;
            rep["init_ms"] = initMs;
            rep["update_ms"] = std::move(updateMs);
            rep["frame_ms"] = std::move(frameMs);
            rep["non_finite"] = inv.nonFinite;
            repeats.push_back(std::move(rep));
        }
        out["repeats"] = std::move(repeats);
        std::ofstream(o.out) << out.dump();
        return 0;
    }

} // namespace

int main(int argc, char** argv)
{
    try {
        const Options o = parse(argc, argv);
        if (o.mode == "simulate")
            return runSimulate(o);
        if (o.mode == "reference")
            return runReference(o);
        return runBench(o);
    }
    catch (const std::exception& e) {
        std::cerr << "physics_qa_runner: " << e.what() << "\n";
        return 1;
    }
}
