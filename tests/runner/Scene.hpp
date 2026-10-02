#pragma once

#include <fstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <nlohmann/json.hpp>
#include <types/World.hpp>

namespace qa {

    namespace detail {
        /**
         * Append the Radius component of one entity. Templated so that the
         * `requires` check is a substitution failure, not a hard error, on a
         * Common version whose WorldState has no radius column.
         */
        template <typename World>
        void pushRadius(World& world, const nlohmann::json& components)
        {
            if constexpr (requires { world.radius; }) {
                typename std::remove_cvref_t<decltype(world.radius)>::value_type radius{};
                if (components.contains("Radius"))
                    radius.value = components.at("Radius").at("value").get<float>();
                world.radius.push_back(radius);
            }
        }
    } // namespace detail

    /**
     * @brief Load a scene using the same JSON layout as the engine's scenes/ JSON files.
     *
     * Only the components the physics module consumes are read: Mass,
     * Position, Velocity and (optionally) Acceleration and Radius. Entities
     * get ids 0..N-1 in file order.
     *
     * Every per-entity vector of the WorldState is filled to the same length
     * so the module can index any of them without bounds checks. The radius
     * column only exists in Common >= v0.1.2 (collision support); it is
     * filled when present so the runner still builds against older Common
     * checkouts (`--local-common`). A missing Radius component defaults to
     * 0, i.e. a point mass that never collides.
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
            detail::pushRadius(world, c);
        }
        return world;
    }

} // namespace qa
