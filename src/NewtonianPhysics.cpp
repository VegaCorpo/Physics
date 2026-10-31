#include "NewtonianPhysics.hpp"
#include <algorithm>
#include <cstddef>
#include <expected>
#include <types/types.hpp>
#include <utility>
#include "components/NewtonianState.hpp"
#include "forces/Gravity.hpp"
#include "integration/IIntegrator.hpp"
#include "integration/TimeStep.hpp"
#include "types/World.hpp"
#include "utils/rotation.hpp"

//? Public methods

void physics::NewtonianPhysics::init(common::SpecificDataPhysics world)
{
    this->_worldState = std::move(world);
    this->_newtonianState.syncIn(this->_worldState);
    this->_gravity.setMode(forces::GravityMode::BarnesHut);
}

void physics::NewtonianPhysics::update(double dt)
{
    const auto integrator = this->_integrators.find(this->_worldState.integrator);

    if (!integrator.has_value())
        return;
    const std::expected<integration::SubSteps, integration::TimeStepError> subSteps =
        _timeStepOf(integrator->get()).split(dt);

    if (!this->_canAdvance(subSteps))
        return;
    this->_newtonianState.syncIn(this->_worldState);
    this->_advance(integrator->get(), *subSteps);
    common::rotation::advanceAll(this->_worldState.orientations, this->_worldState.angularVelocities, dt);

    this->_collider.checkCollisions(this->_newtonianState, this->_gravity.octree());
    this->_newtonianState.syncOut(this->_worldState);
}

double physics::NewtonianPhysics::stepDuration() const
{
    const auto integrator = this->_integrators.find(this->_worldState.integrator);

    if (!integrator.has_value())
        return integration::TimeStep::DEFAULT_MAX_SUB_STEP;
    return _timeStepOf(integrator->get()).maxSubStep();
}

void physics::NewtonianPhysics::shutdown()
{}

//? Private methods

void physics::NewtonianPhysics::syncIn(common::SpecificDataPhysics world)
{
    this->_worldState = std::move(world);
}

common::WorldState physics::NewtonianPhysics::publish()
{
    common::WorldState world;
    const std::size_t count = std::min({this->_worldState.entitiesId.size(), this->_worldState.positions.size(),
                                        this->_worldState.velocities.size(), this->_worldState.accelerations.size()});

    world.entitiesId.resize(count);
    world.positions.resize(count);
    world.velocities.resize(count);
    world.accelerations.resize(count);

    for (std::size_t i = 0; i < count; i += 1) {
        world.positions[i] = this->_worldState.positions[i];
        world.entitiesId[i] = this->_worldState.entitiesId[i];
        world.accelerations[i] = this->_worldState.accelerations[i];
        world.velocities[i] = this->_worldState.velocities[i];
    }
    this->_publishOrientations(world, count);
    return world;
}

bool physics::NewtonianPhysics::_canAdvance(
    const std::expected<integration::SubSteps, integration::TimeStepError>& subSteps) const
{
    return this->_state == common::ModuleState::RUNNING && subSteps.has_value() && !subSteps->empty();
}

physics::integration::TimeStep physics::NewtonianPhysics::_timeStepOf(const integration::IIntegrator& integrator)
{
    return integration::TimeStep::forForceEvaluations(integrator.forceEvaluationsPerStep())
        .value_or(integration::TimeStep{});
}

void physics::NewtonianPhysics::_advance(const integration::IIntegrator& integrator,
                                         const integration::SubSteps& subSteps)
{
    const integration::Bodies bodies{this->_worldState, this->_newtonianState};

    for (std::size_t stepIndex = 0; stepIndex < subSteps.count; stepIndex += 1)
        integrator.step(bodies, this->_gravity, subSteps.duration);
}

void physics::NewtonianPhysics::_publishOrientations(common::WorldState& world, std::size_t count) const
{
    const auto& orientations = this->_worldState.orientations;

    world.orientations.resize(count);
    std::copy_n(orientations.begin(), std::min(count, orientations.size()), world.orientations.begin());
}
