#include "NewtonianPhysics.hpp"
#include <utility>
#include "forces/Gravity.hpp"
#include "integration/Verlet.hpp"
#include "types/World.hpp"

//? Public methods

void physics::NewtonianPhysics::init(common::SpecificDataPhysics world)
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

void physics::NewtonianPhysics::syncIn(common::SpecificDataPhysics world)
{
    this->_world_state = world;
}

common::WorldState physics::NewtonianPhysics::publish()
{
    common::WorldState world;
    const std::size_t count = std::min(
                    {this->_world_state.entitiesId.size(), this->_world_state.positions.size(), this->_world_state.velocities.size(), this->_world_state.accelerations.size()});
    
    world.entitiesId.resize(count);
    world.positions.resize(count);
    world.velocities.resize(count);
    world.accelerations.resize(count);
    
    for (std::size_t i = 0; i < count; i += 1) {
        world.positions[i] = this->_world_state.positions[i];
        world.entitiesId[i] = this->_world_state.entitiesId[i];
        world.accelerations[i] = this->_world_state.accelerations[i];
        world.velocities[i] = this->_world_state.velocities[i];
    }
    return world;
}
