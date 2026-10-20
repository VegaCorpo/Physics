#pragma once

#include <expected>
#include <functional>
#include <memory>
#include <types/types.hpp>
#include <unordered_map>
#include "integration/IIntegrator.hpp"

namespace physics::integration {
    enum class IntegratorError {
        UNKNOWN_INTEGRATOR,
    };

    class IntegratorCatalog {
        public:
            IntegratorCatalog();

            [[nodiscard]] std::expected<std::reference_wrapper<const IIntegrator>, IntegratorError>
            find(common::IntegratorType type) const;

        private:
            std::unordered_map<common::IntegratorType, std::unique_ptr<IIntegrator>> _integrators;
    };
} // namespace physics::integration
