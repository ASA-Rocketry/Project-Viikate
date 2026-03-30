#include "state_machine.h"
#include "constants.h"

StateMachine::StateMachine() {
    active_state = State::kIdle;
}

State StateMachine::GetState() {
    return active_state;
}

bool StateMachine::StagingCheck(const FlightData& data) const{
    if (data.rbfRemoved && (data.accZ < 1.0f)) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::LiftoffCheck(const FlightData& data) const{
    if (data.accZ > (1.5f * constants::kGravity)) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::BurnoutCheck(const FlightData& data) const{
    if ((data.accZ < 0.0f)) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::ApogeeCheck(const FlightData& data) const{
    if (data.verticalVelocity < 0.0f) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::RDDCheck(const FlightData& data) const{
    if (((data.verticalVelocity < 5.0f) && (data.accZ < 0.0f)) ||
        (data.timeMs - liftoff_time_ms > 20000)) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::LandCheck(const FlightData& data) const{
    if (((data.verticalVelocity > -1.0f && data.verticalVelocity < 1.0f)) &&
        (data.accelMagnitude < 1.0f)) {
        return true;
    } else {
        return false;
    }
}

State StateMachine::Update(const FlightData& data){
    State next_state = active_state;

        switch (active_state) {
            case State::kIdle:
                if (StagingCheck(data)) next_state = State::kLaunchpad;
                break;

            case State::kLaunchpad:
                if (LiftoffCheck(data)) next_state = State::kLiftoff;
                break;

            case State::kLiftoff:
                if (BurnoutCheck(data)) next_state = State::kCoast;
                break;

            case State::kCoast:
                if (ApogeeCheck(data)) next_state = State::kApogee;
                break;

            case State::kApogee:
                if (RDDCheck(data)) next_state = State::kRDD;
                break;

            case State::kRDD:
                if (LandCheck(data)) next_state = State::kGround;
                break;

            default:
                break;
        }

        if (next_state != active_state) {
        OnEnter(next_state, data.timeMs);
        active_state = next_state;
    }
    return active_state;
}

void StateMachine::OnEnter(State new_state, unsigned long time_ms) {
    state_entry_time_ms = time_ms;
    
    switch(new_state) {
        case State::kIdle:
            break;

        case State::kLaunchpad:
            break;

        case State::kLiftoff:
            liftoff_time_ms = time_ms;
            break;

        case State::kCoast:
            break;

        case State::kApogee:
            break;

        case State::kRDD:
            break;

        case State::kGround:
            break;

        default:
            break;
    }
}