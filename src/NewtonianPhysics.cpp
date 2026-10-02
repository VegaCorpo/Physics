#include "NewtonianPhysics.hpp"
#include <algorithm>
#include <utility>
#include "components/NewtonianState.hpp"
#include "forces/Gravity.hpp"
#include "integration/Verlet.hpp"
#include "types/World.hpp"
#include "utils/rotation.hpp"

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
    common::rotation::advanceAll(this->_worldState.orientations, this->_worldState.angularVelocities, dt);

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
    this->_publishOrientations(world, count);
    return world;
}

void physics::NewtonianPhysics::_publishOrientations(common::WorldState& world, std::size_t count) const
{
    const auto& orientations = this->_worldState.orientations;

    world.orientations.resize(count);
    std::copy_n(orientations.begin(), std::min(count, orientations.size()), world.orientations.begin());
}
