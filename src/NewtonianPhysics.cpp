#include "NewtonianPhysics.hpp"
#include <utility>
#include "forces/Gravity.hpp"
#include "integration/Verlet.hpp"

//? Public methods

void physics::NewtonianPhysics::init(common::WorldState world)
{
    this->_world_state = std::move(world);
}

void physics::NewtonianPhysics::update(double dt)
{
    integration::Verlet::preIntegrate(this->_world_state, dt);
    forces::Gravity::apply(this->_world_state, dt);
    integration::Verlet::postIntegrate(this->_world_state, dt);
}

void physics::NewtonianPhysics::shutdown()
{}

//? Private methods

void physics::NewtonianPhysics::syncIn(common::WorldState world)
{
    this->_world_state = world;
    // this->_syncPositionToPhysics(registry);
    // this->_syncVelocityToPhysics(registry);
    // this->_syncAccelerationToPhysics(registry);
    // this->_syncMassToPhysics(registry);
}

common::WorldState physics::NewtonianPhysics::syncOut()
{
    return this->_world_state;
}
