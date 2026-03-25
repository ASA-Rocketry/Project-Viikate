#include "state_machine.h"
#include "constants.h"

StateMachine::StateMachine() {
    ActiveState = IDLE;
}

State StateMachine::GetState() {
    return ActiveState;
}

bool StateMachine::StagingCheck(const FlightData& data) const{
    if (data.rbfRemoved and (data.accelZ < 1)) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::LiftoffCheck(const FlightData& data) const{
    if (data.accelZ > (1.5 * 9.81)) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::BurnoutCheck(const FlightData& data) const{
    if ((data.accelZ < 0)) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::ApogeeCheck(const FlightData& data) const{
    if (data.verticalVelocity < 0) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::RDDCheck(const FlightData& data) const{
    if (((data.verticalVelocity < 5) and (data.accelZ < 0)) or (data.timeMs - liftoffTimeMs > 20000)) {
        return true;
    } else {
        return false;
    }
}

bool StateMachine::LandCheck(const FlightData& data) const{
    if (((data.verticalVelocity > -1 && data.verticalVelocity < 1)) and (data.accelMagnitude < 1)) {
        return true;
    } else {
        return false;
    }
}

State StateMachine::update(const FlightData& data){
    State nextState = ActiveState;

        switch (ActiveState) {
            case IDLE:
                if (StagingCheck(data)) nextState = LAUNCHPAD;
                break;

            case LAUNCHPAD:
                if (LiftoffCheck(data)) nextState = LIFTOFF;
                break;

            case LIFTOFF:
                if (BurnoutCheck(data)) nextState = COAST;
                break;

            case COAST:
                if (ApogeeCheck(data)) nextState = APOGEE;
                break;

            case APOGEE:
                if (RDDCheck(data)) nextState = RDD;
                break;

            case RDD:
                if (LandCheck(data)) nextState = GROUND;
                break;

            default:
                break;
        }

        if (nextState != ActiveState) {
        OnEnter(nextState, data.timeMs);
        ActiveState = nextState;
    }
    return ActiveState;
}

void StateMachine::OnEnter(State newState, unsigned long timeMs) {
    stateEntryTimeMs = timeMs;
    
    switch(newState) {
        case IDLE:
            break;

        case LAUNCHPAD:
            break;

        case LIFTOFF:
            liftoffTimeMs = timeMs;
            break;

        case COAST:
            break;

        case APOGEE:
            break;

        case RDD:
            break;

        case GROUND:
            break;

        default:
            break;
    }
}
