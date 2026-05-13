#include <Arduino.h>
#include "state_machine.h"
#include "control.h"
#include "constants.h"


const char* StateToString(State state) {
    switch (state) {
        case State::kCalibration: return "Calibration: sensor calibration in progress";
        case State::kIdle: return "Idle: armed check, no flight activity";
        case State::kLaunchpad: return "Launchpad: RBF removed, waiting for ignition";
        case State::kLiftoff: return "Liftoff: motor burning, strong upward acceleration";
        case State::kCoast: return "Coast: burnout, upward velocity slowing";
        case State::kApogee: return "Apogee: vertical velocity near zero";
        case State::kRDD: return "RDD: descent/recovery device deployment";
        case State::kGround: return "Ground: landed and stopped";
        default: return "Unknown";
    }
}

StateMachine::StateMachine(DataLogger& data_logger, Control& control): data_logger_(data_logger), control_(control){
    active_state = State::kCalibration;
}

State StateMachine::GetState() { return active_state; }

bool StateMachine::IdleCheck(const FlightData& data) const{
    if (data_logger_.IsInitialized() /*&& sensors_.IsInitialized()*/ && control_.IsInitialized()) {
        return true;
    } else {
        return false;
    }
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
    if ((abs(data.verticalVelocity) < 1.0f) && (data.accelMagnitude < 1.0f)) { // TODO: Probably needs a time based check as well?
        return true;
    } else {
        return false;
    }
}

State StateMachine::Update(const FlightData& data){
    State next_state = active_state;
        switch (active_state) {
            case State::kCalibration:
                if (IdleCheck(data)) next_state = State::kIdle;
                break;

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
#ifdef TEST_STATE_MACHINE_MODE
            Serial.print("Exiting state: ");
            Serial.println(StateToString(active_state));
            Serial.print("Entering state: ");
            Serial.println(StateToString(next_state));
#endif
            OnEnter(next_state, data.timeMs);
            active_state = next_state;
    }
    return active_state;
}

void StateMachine::OnEnter(State new_state, unsigned long time_ms) {
    state_entry_time_ms = time_ms;
    
    switch(new_state) {
        case State::kCalibration:
            data_logger_.LogEvent(LogType::kInfo, "State: Calibration");
            break;
        
        case State::kIdle:
            data_logger_.LogEvent(LogType::kInfo, "State: Idle");
            break;

        case State::kLaunchpad:
            data_logger_.LogEvent(LogType::kInfo, "State: Launchpad");
            break;

        case State::kLiftoff:
            data_logger_.LogEvent(LogType::kInfo, "State: Liftoff");
            liftoff_time_ms = time_ms;
            break;

        case State::kCoast:
            data_logger_.LogEvent(LogType::kInfo, "State: Coast");
            break;

        case State::kApogee:
            data_logger_.LogEvent(LogType::kInfo, "State: Apogee");
            break;

        case State::kRDD:
            data_logger_.LogEvent(LogType::kInfo, "State: Recovery");
            break;

        case State::kGround:
            data_logger_.LogEvent(LogType::kInfo, "State: Ground");
            delay(2000); // Wait for 2 seconds to ensure all flight data is logged before closing files
            data_logger_.Close(); // Close SD card
            break;

        default:
            break;
    }
}