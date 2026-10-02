#include "NewtonianPhysics.hpp"
#include <utility>
#include "components/NewtonianState.hpp"
#include "forces/Gravity.hpp"
#include "integration/Verlet.hpp"
#include "types/World.hpp"

//? Public methods

void physics::NewtonianPhysics::init(common::SpecificDataPhysics world)
{
    this->_worldState = std::move(world);
    this->_newtonianState.syncIn(this->_worldState);
}

void physics::NewtonianPhysics::update(double dt)
{
    this->_newtonianState.syncIn(this->_worldState);

    if (this->_newtonianState.forcesStale()) {
        this->_octree.build(this->_newtonianState);
        forces::Gravity::apply(this->_newtonianState, dt);
        this->_newtonianState.markForcesFresh();
    }

    integration::Verlet::preIntegrate(this->_worldState, this->_newtonianState, dt);
    this->_octree.build(this->_newtonianState);
    forces::Gravity::apply(this->_newtonianState, dt);
    integration::Verlet::postIntegrate(this->_worldState, this->_newtonianState, dt);

    this->_collider.checkCollisions(this->_newtonianState, this->_octree);
    this->_newtonianState.syncOut(this->_worldState);
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
    return world;
}
