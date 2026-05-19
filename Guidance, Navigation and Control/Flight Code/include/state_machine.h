#ifndef FLIGHT_CODE_INCLUDE_STATE_MACHINE_H_
#define FLIGHT_CODE_INCLUDE_STATE_MACHINE_H_

#include "constants.h"
#include "data_logger.h"
#include "control.h"
#include "sensors.h"

/**
 * @file state_machine.h
 * @brief Discrete flight states for the rocket.
 *
 * The state machine progresses monotonically through these states.
 * States should NEVER be skipped backward.
 */
enum class State {
    kCalibration,  // Sensor calibration in progress
    kIdle,        // Rocket is not armed, RBF installed and no flight activity
    kLaunchpad,   // RBF removed, armed and waiting for ignition
    kLiftoff,     // Motor burning, strong upward acceleration
    kCoast,       // Motor burnout, upward velocity and downward acceleration
    kApogee,      // Vertical velocity ~ 0 (transition state)
    kRDD,         // Descent detected (5 m/s), recovery device deployed
    kGround       // Rocket has landed and stopped moving
};

/**
 * @brief Convert a State enum to a string representation.
 * @param state The state to convert.
 * @return const char* String representation of the state.
 */
const char* StateToString(State state);

/**
 * @brief Rocket flight state machine.
 *
 * Responsible ONLY for determining the current flight state
 * based on FlightData inputs. Has no direct authority over data logging,
 * hardware or RDD.
 */
class StateMachine {
public:
    /**
     * @brief Construct a new StateMachine object.
     *
     * Initializes the active state to kIdle.
     */
    StateMachine(DataLogger& data_logger, Sensors& sensors, Control& control);

    /**
     * @brief Update the StateMachine instance with new flight data.
     * 
     * This function is called once per main loop iteration. 
     * It evaluates transition conditions given current state and provided 
     * FlightData
     * 
     * @param data  Latest fused flight data from hardware and sensors
     * @return State  The updated flight state
     */
    State Update(const FlightData& data);

    /**
     * @brief Get the current state
     * 
     * @return State Current flight state
     */
    State GetState();

private:
    /**
     * @brief Currently active flight state.
     */
    State active_state;

    /**
     * @brief Timestamp when the current state was entered.
     *
     * Used for time-based gating and safety checks.
     */
    unsigned long state_entry_time_ms;

    /**
     * @brief Timestamp when liftoff occurred
     * 
     * Portrayed as ms from boot
     */
    unsigned long liftoff_time_ms;

    /* ---------- State transition condition checks ---------- */

    /**
     * @brief Check transition from kCalibration to kIdle.
     *
     * Typical conditions:
     *  - All subsystems report successful initialization
     */ bool IdleCheck(const FlightData& data) const;

    /**
     * @brief Check transition from kIdle to kLaunchpad.
     *
     * Typical conditions:
     *  - RBF pin removed
     *  - No significant acceleration
     */
    bool StagingCheck(const FlightData& data) const;

    /**
     * @brief Check transition from kLaunchpad to kLiftoff.
     *
     * Typical conditions:
     *  - Sustained upward acceleration above threshold
     */
    bool LiftoffCheck(const FlightData& data) const;

    /**
     * @brief Check transition from kLiftoff to kCoast.
     *
     * Typical conditions:
     *  - Acceleration drops below 0 or near free-fall
     *  - Minimum burn time satisfied
     */
    bool BurnoutCheck(const FlightData& data) const;

    /**
     * @brief Check transition from kCoast to kApogee.
     *
     * Typical conditions:
     *  - Vertical velocity crosses zero (within tolerance)
     */
    bool ApogeeCheck(const FlightData& data) const;

    /**
     * @brief Check transition from kApogee to kRdd (descent).
     *
     * Typical conditions:
     *  - Negative vertical velocity beyond threshold
     *  - Acceleration indicates downward motion / free-fall
     * OR
     *  - Max theoretical flight time reached
     */
    bool RDDCheck(const FlightData& data) const;

    /**
     * @brief Check transition from kRdd to kGround.
     *
     * Typical conditions:
     *  - Vertical velocity ~ 0
     *  - Acceleration magnitude low
     */
    bool LandCheck(const FlightData& data) const;

    /**
     * @brief Handle state entry actions.
     *
     * Called exactly once when entering a new state.
     * Use this for resetting timers, latching events, etc.
     */
    void OnEnter(State newState, unsigned long timeMs);

    DataLogger& data_logger_; 
    Sensors& sensors_;
    Control& control_;
};

#endif  // FLIGHT_CODE_INCLUDE_STATE_MACHINE_H_