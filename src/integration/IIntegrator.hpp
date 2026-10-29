#pragma once

#include <cstddef>
#include <types/World.hpp>
#include "components/NewtonianState.hpp"

namespace physics::integration {
    struct Bodies {
            common::SpecificDataPhysics& world;
            NewtonianState& state;
    };

    class IAccelerationField {
        public:
            virtual ~IAccelerationField() = default;

            virtual void compute(NewtonianState& state) = 0;
    };

    class IIntegrator {
        public:
            virtual ~IIntegrator() = default;

            virtual void step(const Bodies& bodies, IAccelerationField& field, double duration) const = 0;
            [[nodiscard]] virtual std::size_t forceEvaluationsPerStep() const noexcept = 0;
    };
} // namespace physics::integration
