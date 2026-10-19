#pragma once

#include "components/NewtonianState.hpp"
#include "forces/Gravity.hpp"
#include "integration/IIntegrator.hpp"
#include "spatial/Octree.hpp"

namespace physics::forces {
    class GravityField : public integration::IAccelerationField {
        public:
            void compute(NewtonianState& state) override
            {
                this->_octree.build(state);
                Gravity::apply(state, this->_mode, this->_octree);
            }

            [[nodiscard]] GravityMode mode() const noexcept { return this->_mode; }

            void setMode(GravityMode mode) noexcept { this->_mode = mode; }

            [[nodiscard]] Octree& octree() noexcept { return this->_octree; }

        private:
            Octree _octree;
            GravityMode _mode = GravityMode::BarnesHut;
    };
} // namespace physics::forces
