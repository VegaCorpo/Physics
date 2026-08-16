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
    forces::Gravity::apply(this->_world_state, this->_newtonian_state, dt);
    integration::Verlet::postIntegrate(this->_world_state, dt);
}

void physics::NewtonianPhysics::shutdown()
{}

//? Private methods

void physics::NewtonianPhysics::syncIn(common::WorldState world)
{
    this->_world_state = world;
}

common::WorldState physics::NewtonianPhysics::syncOut()
{
    return this->_world_state;
}
