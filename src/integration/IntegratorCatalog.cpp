#include "IntegratorCatalog.hpp"
#include "integration/Pefrl.hpp"
#include "integration/Verlet.hpp"

physics::integration::IntegratorCatalog::IntegratorCatalog()
{
    this->_integrators.emplace(common::IntegratorType::VERLET, std::make_unique<Verlet>());
    this->_integrators.emplace(common::IntegratorType::PEFRL, std::make_unique<Pefrl>());
}

auto physics::integration::IntegratorCatalog::find(common::IntegratorType type) const
    -> std::expected<std::reference_wrapper<const IIntegrator>, IntegratorError>
{
    const auto found = this->_integrators.find(type);

    if (found == this->_integrators.end())
        return std::unexpected(IntegratorError::UNKNOWN_INTEGRATOR);
    return std::cref(*found->second);
}
