#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <interfaces/IPhysicsEngine.hpp>
#include <types/World.hpp>

namespace qa {

    /**
     * Everything that differs between the versions of the frame contract lives
     * here, so the rest of the runner builds against the Common pinned by any
     * checkout of the module:
     *
     *   Common <  v0.2.0  init/syncIn(WorldState), WorldState syncOut()
     *                     columns `entities` and `mass`
     *   Common >= v0.2.0  init/syncIn(SpecificDataPhysics), WorldState publish()
     *                     columns `entitiesId` and `masses`; publish() carries
     *                     no mass, so its result is merged back into the input
     *
     * Every helper is a template so that members missing from one version are
     * substitution failures in a discarded branch, not hard errors.
     */

    namespace detail {
        template <typename>
        struct ParameterOf;

        template <typename Class, typename Result, typename Argument>
        struct ParameterOf<Result (Class::*)(Argument)> {
                using type = std::remove_cvref_t<Argument>;
        };
    } // namespace detail

    /// What the module consumes in init() and syncIn().
    using State = detail::ParameterOf<decltype(&common::IPhysicsEngine::init)>::type;

    template <typename World>
    auto& idsOf(World& world)
    {
        if constexpr (requires { world.entitiesId; })
            return world.entitiesId;
        else
            return world.entities;
    }

    template <typename World>
    auto& massesOf(World& world)
    {
        if constexpr (requires { world.masses; })
            return world.masses;
        else
            return world.mass;
    }

    /// End of a frame: publish() on the current contract, syncOut() on the old one.
    template <typename Engine>
    auto collect(Engine& engine)
    {
        if constexpr (requires { engine.publish(); })
            return engine.publish();
        else
            return engine.syncOut();
    }

    /**
     * @brief Fold what collect() returned into the state fed to the next frame.
     *
     * The old syncOut() returned the whole state. publish() returns positions,
     * velocities, accelerations and orientations only, so they are copied back
     * the way Core's PhysicsSync::scatter does, while masses, radii and angular
     * velocities stay those of the input. The published entities must be the
     * ones sent, in the same order: anything else (lost, duplicated or
     * reordered bodies) is a module bug and is reported as an error.
     */
    template <typename World, typename Output>
    void apply(World& state, Output&& out)
    {
        if constexpr (std::is_same_v<std::remove_cvref_t<Output>, World>) {
            state = std::forward<Output>(out);
        } else {
            const auto& ids = idsOf(state);
            const std::size_t count = std::min(
                {out.entitiesId.size(), out.positions.size(), out.velocities.size(), out.accelerations.size()});

            if (count != ids.size() || !std::equal(ids.begin(), ids.end(), out.entitiesId.begin()))
                throw std::runtime_error("publish() returned " + std::to_string(count) + " consistent entities, " +
                                         std::to_string(ids.size()) + " expected in the order they were sent");

            for (std::size_t i = 0; i < count; i += 1) {
                state.positions[i] = out.positions[i];
                state.velocities[i] = out.velocities[i];
                state.accelerations[i] = out.accelerations[i];
            }
            if constexpr (requires { out.orientations; state.orientations; }) {
                const std::size_t oriented = std::min(out.orientations.size(), state.orientations.size());
                std::copy_n(out.orientations.begin(), oriented, state.orientations.begin());
            }
        }
    }

} // namespace qa
