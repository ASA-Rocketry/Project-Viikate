#include "state_machine.h"
#include "constants.h"

/**
 * @brief Constructs a new StateMachine object.
 *
 * Initializes the active state to kIdle.
 */
StateMachine::StateMachine() : active_state_(State::kIdle),
                               state_entry_time_ms_(0),
                               liftoff_time_ms_(0) {}

/**
 * @brief Returns the currently active state.
 * @return State Current flight state.
 */
State StateMachine::GetState() const {
  return active_state_;
}

/* ---------- State transition checks ---------- */

bool StateMachine::StagingCheck(const FlightData& data) const {
  return (data.rbf_removed && (data.accel_z < 1.0f));
}

bool StateMachine::LiftoffCheck(const FlightData& data) const {
  return (data.accel_z > (1.5f * constants::kGravity));
}

bool StateMachine::BurnoutCheck(const FlightData& data) const {
  return (data.accel_z < 0.0f);
}

bool StateMachine::ApogeeCheck(const FlightData& data) const {
  return (data.vertical_velocity < 0.0f);
}

bool StateMachine::RDDCheck(const FlightData& data) const {
  return ((data.vertical_velocity < 5.0f && data.accel_z < 0.0f) ||
          (data.time_ms - liftoff_time_ms_ > 20000));
}

bool StateMachine::LandCheck(const FlightData& data) const {
  return ((data.vertical_velocity > -1.0f && data.vertical_velocity < 1.0f) &&
          (data.accel_magnitude < 1.0f));
}

/**
 * @brief Updates the state machine based on the latest flight data.
 * @param data Latest fused flight data.
 * @return State The updated flight state.
 */
State StateMachine::Update(const FlightData& data) {
  State next_state = active_state_;

  switch (active_state_) {
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
      if (RDDCheck(data)) next_state = State::kRdd;
      break;

    case State::kRdd:
      if (LandCheck(data)) next_state = State::kGround;
      break;

    case State::kGround:
      break;
  }

  if (next_state != active_state_) {
    OnEnter(next_state, data.time_ms);
    active_state_ = next_state;
  }

  return active_state_;
}

/**
 * @brief Handles actions when entering a new state.
 * @param new_state The state being entered.
 * @param time_ms Current timestamp in milliseconds.
 */
void StateMachine::OnEnter(State new_state, unsigned long time_ms) {
  state_entry_time_ms_ = time_ms;

  switch (new_state) {
    case State::kIdle:
      break;

    case State::kLaunchpad:
      break;

    case State::kLiftoff:
      liftoff_time_ms_ = time_ms;
      break;

    case State::kCoast:
      break;

    case State::kApogee:
      break;

    case State::kRdd:
      break;

    case State::kGround:
      break;
  }
}

// Test