#include "Verlet.hpp"
#include "integration/Motion.hpp"

void physics::integration::Verlet::step(const Bodies& bodies, IAccelerationField& field, double duration) const
{
    const double halfDuration = HALF * duration;

    _refreshStaleForces(bodies.state, field);
    motion::kick(bodies, halfDuration);
    motion::drift(bodies, duration);
    field.compute(bodies.state);
    motion::kick(bodies, halfDuration);
}

void physics::integration::Verlet::_refreshStaleForces(NewtonianState& state, IAccelerationField& field)
{
    if (!state.forcesStale())
        return;
    field.compute(state);
    state.markForcesFresh();
}
