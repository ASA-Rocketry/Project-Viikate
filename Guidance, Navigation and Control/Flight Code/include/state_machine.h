#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include "constants.h"

/**
 * @file state_machine.h
 * @brief Discrete flight states and state machine for the rocket.
 *
 * The state machine progresses monotonically through these states.
 * States should never be skipped backward.
 */
enum class State {
  kIdle,       /**< Rocket is not armed, RBF installed, no flight activity. */
  kLaunchpad,  /**< RBF removed, armed, and waiting for ignition. */
  kLiftoff,    /**< Motor burning, strong upward acceleration. */
  kCoast,      /**< Motor burnout, upward velocity and downward acceleration. */
  kApogee,     /**< Vertical velocity approximately 0 (transition state). */
  kRdd,        /**< Descent detected, recovery device deployed. */
  kGround      /**< Rocket has landed and stopped moving. */
};

/**
 * @class StateMachine
 * @brief Rocket flight state machine.
 *
 * Determines the current flight state based on FlightData inputs.
 * Does not control hardware, data logging, or RDD directly.
 */
class StateMachine {
 public:
  /** 
   * @brief Constructs a new StateMachine object.
   *
   * Initializes the active state to kIdle.
   */
  StateMachine();

  /**
   * @brief Updates the state machine with new flight data.
   *
   * Called once per main loop iteration. Evaluates transition conditions
   * based on the current state and the provided FlightData.
   *
   * @param data Latest fused flight data from hardware and sensors.
   * @return State The updated flight state.
   */
  State Update(const FlightData& data);

  /**
   * @brief Returns the currently active state.
   * @return State Current flight state.
   */
  State GetState() const;

 private:
  /** @brief Currently active flight state. */
  State active_state_;

  /** @brief Timestamp when the current state was entered (ms from boot). */
  unsigned long state_entry_time_ms_;

  /** @brief Timestamp when liftoff occurred (ms from boot). */
  unsigned long liftoff_time_ms_;

  /* ---------- State transition condition checks ---------- */

  /** @brief Check transition from kIdle to kLaunchpad. */
  bool StagingCheck(const FlightData& data) const;

  /** @brief Check transition from kLaunchpad to kLiftoff. */
  bool LiftoffCheck(const FlightData& data) const;

  /** @brief Check transition from kLiftoff to kCoast. */
  bool BurnoutCheck(const FlightData& data) const;

  /** @brief Check transition from kCoast to kApogee. */
  bool ApogeeCheck(const FlightData& data) const;

  /** @brief Check transition from kApogee to kRdd. */
  bool RDDCheck(const FlightData& data) const;

  /** @brief Check transition from kRdd to kGround. */
  bool LandCheck(const FlightData& data) const;

  /**
   * @brief Handle actions upon entering a new state.
   *
   * Called exactly once when entering a new state. Used for timers,
   * event latching, or other state-specific setup.
   *
   * @param new_state The state being entered.
   * @param time_ms Current timestamp in milliseconds.
   */
  void OnEnter(State new_state, unsigned long time_ms);
};

#endif  // STATE_MACHINE_H_

// Test