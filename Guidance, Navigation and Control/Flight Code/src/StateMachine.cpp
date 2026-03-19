#include "StateMachine.h"
#include "Constants.h"

StateMachine::StateMachine() {
    ActiveState_ = IDLE;
}

State StateMachine::GetState() {
    return ActiveState_;
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
    if (((data.verticalVelocity < 5) and (data.accelZ < 0)) or (data.timeMs - LiftoffTimeMs_ > 20000)) {
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

State StateMachine::Update(const FlightData& data){
    State nextState = ActiveState_;

        switch (ActiveState_) {
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

        if (nextState != ActiveState_) {
        OnEnter(nextState, data.timeMs);
        ActiveState_ = nextState;
    }
    return ActiveState_;
}

void StateMachine::OnEnter(State newState, unsigned long timeMs) {
    StateEntryTimeMs_ = timeMs;
    
    switch(newState) {
        case IDLE:
            break;

        case LAUNCHPAD:
            break;

        case LIFTOFF:
            LiftoffTimeMs_ = timeMs;
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
