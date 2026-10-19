#pragma once

#include <map>
#include <stdexcept>
#include <string>

namespace qa {

    namespace detail {
        template <typename Integrator>
        Integrator integratorNamed(const std::string& name)
        {
            const std::map<std::string, Integrator> known{{"verlet", Integrator::VERLET}, {"pefrl", Integrator::PEFRL}};
            const auto found = known.find(name);

            if (found == known.end())
                throw std::invalid_argument("unknown integrator '" + name + "' (expected verlet or pefrl)");
            return found->second;
        }
    } // namespace detail

    template <typename World>
    void selectIntegrator(World& world, const std::string& name)
    {
        if (name.empty())
            return;
        if constexpr (requires { world.integrator; })
            world.integrator = detail::integratorNamed<decltype(world.integrator)>(name);
        else
            throw std::runtime_error("--integrator needs a Common version with SpecificDataPhysics::integrator");
    }

} // namespace qa
