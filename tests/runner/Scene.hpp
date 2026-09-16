#pragma once

#include <fstream>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include <types/World.hpp>

namespace qa {

    /**
     * @brief Load a scene using the same JSON layout as the engine's scenes/ JSON files.
     *
     * Only the components the physics module consumes are read: Mass,
     * Position, Velocity and (optionally) Acceleration. Entities get ids
     * 0..N-1 in file order.
     */
    inline common::WorldState loadScene(const std::string& path)
    {
        std::ifstream in(path);
        if (!in)
            throw std::runtime_error("cannot open scene " + path);

        const nlohmann::json doc = nlohmann::json::parse(in);
        common::WorldState world;

        for (const auto& entity : doc.at("entities")) {
            const auto& c = entity.at("components");

            common::components::Position p;
            p.x = c.at("Position").at("x").get<double>();
            p.y = c.at("Position").at("y").get<double>();
            p.z = c.at("Position").at("z").get<double>();

            common::components::Velocity v;
            v.x = c.at("Velocity").at("x").get<double>();
            v.y = c.at("Velocity").at("y").get<double>();
            v.z = c.at("Velocity").at("z").get<double>();

            common::components::Acceleration a;
            if (c.contains("Acceleration")) {
                a.x = c.at("Acceleration").at("x").get<double>();
                a.y = c.at("Acceleration").at("y").get<double>();
                a.z = c.at("Acceleration").at("z").get<double>();
            }

            common::components::Mass m;
            m.mantissa = c.at("Mass").at("mantissa").get<float>();
            m.exponent = c.at("Mass").at("exponent").get<int>();

            world.entities.push_back(world.entities.size());
            world.positions.push_back(p);
            world.velocities.push_back(v);
            world.accelerations.push_back(a);
            world.mass.push_back(m);
        }
        return world;
    }

} // namespace qa
