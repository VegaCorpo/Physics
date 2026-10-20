#include "Pefrl.hpp"
#include "integration/Motion.hpp"

void physics::integration::Pefrl::step(const Bodies& bodies, IAccelerationField& field, double duration) const
{
    motion::drift(bodies, XI * duration);
    _kick(bodies, field, OUTER_KICK * duration);
    motion::drift(bodies, CHI * duration);
    _kick(bodies, field, LAMBDA * duration);
    motion::drift(bodies, MIDDLE_DRIFT * duration);
    _kick(bodies, field, LAMBDA * duration);
    motion::drift(bodies, CHI * duration);
    _kick(bodies, field, OUTER_KICK * duration);
    motion::drift(bodies, XI * duration);
    bodies.state.markForcesStale();
}

void physics::integration::Pefrl::_kick(const Bodies& bodies, IAccelerationField& field, double duration)
{
    field.compute(bodies.state);
    motion::kick(bodies, duration);
}
