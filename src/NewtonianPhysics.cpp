#include "NewtonianPhysics.hpp"
#include <utility>
#include "forces/Gravity.hpp"
#include "integration/Verlet.hpp"

//? Public methods

void physics::NewtonianPhysics::init(common::WorldState world)
{
    this->_world_state = std::move(world);
    this->_newtonian_state.syncIn(this->_world_state);
}

void physics::NewtonianPhysics::update(double dt)
{
    this->_newtonian_state.syncIn(this->_world_state);

    if (this->_newtonian_state.forcesStale()) {
        this->_octree.build(this->_newtonian_state);
        forces::Gravity::apply(this->_newtonian_state, dt);
        this->_newtonian_state.markForcesFresh();
    }

    integration::Verlet::preIntegrate(this->_world_state, this->_newtonian_state, dt);
    this->_octree.build(this->_newtonian_state);
    forces::Gravity::apply(this->_newtonian_state, dt);
    integration::Verlet::postIntegrate(this->_world_state, this->_newtonian_state, dt);

    this->_newtonian_state.syncOut(this->_world_state);
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
