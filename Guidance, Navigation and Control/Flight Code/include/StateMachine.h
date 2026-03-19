#ifndef STATEMACHINE
#define STATEMACHINE

#include "Constants.h"

/**
 * @file StateMachine.h
 * @brief Discrete flight states for the rocket.
 *
 * The state machine progresses monotonically through these states.
 * States should NEVER be skipped backward.
 */
enum State {
    IDLE,        // Rocket is not armed, RBF installed and no flight activity
    LAUNCHPAD,   // RBF removed, armed and waiting for ignition
    LIFTOFF,     // Motor burning, strong upward acceleration
    COAST,       // Motor burnout, upward velocity and downward acceleration
    APOGEE,      // Vertical velocity ~ 0 (transition state)
    RDD,         // Descent detected (5 m/s), recovery device deployed
    GROUND       // Rocket has landed and stopped moving
};



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
     * Initializes the active state to IDLE.
     */
    StateMachine();

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
    State ActiveState_;

    /**
     * @brief Timestamp when the current state was entered.
     *
     * Used for time-based gating and safety checks.
     */
    unsigned long StateEntryTimeMs_;

    /**
     * @brief Timestamp when liftoff occurred
     * 
     * Portrayed as ms from boot
     */
    unsigned long LiftoffTimeMs_;

    /* ---------- State transition condition checks ---------- */

    /**
     * @brief Check transition from IDLE to LAUNCHPAD.
     *
     * Typical conditions:
     *  - RBF pin removed
     *  - No significant acceleration
     */
    bool StagingCheck(const FlightData& data) const;

    /**
     * @brief Check transition from LAUNCHPAD to LIFTOFF.
     *
     * Typical conditions:
     *  - Sustained upward acceleration above threshold
     */
    bool LiftoffCheck(const FlightData& data) const;

    /**
     * @brief Check transition from LIFTOFF to COAST.
     *
     * Typical conditions:
     *  - Acceleration drops below 0 or near free-fall
     *  - Minimum burn time satisfied
     */
    bool BurnoutCheck(const FlightData& data) const;

    /**
     * @brief Check transition from COAST to APOGEE.
     *
     * Typical conditions:
     *  - Vertical velocity crosses zero (within tolerance)
     */
    bool ApogeeCheck(const FlightData& data) const;

    /**
     * @brief Check transition from APOGEE to RDD (descent).
     *
     * Typical conditions:
     *  - Negative vertical velocity beyond threshold
     *  - Acceleration indicates downward motion / free-fall
     * OR
     *  - Max theoretical flight time reached
     */
    bool RDDCheck(const FlightData& data) const;

    /**
     * @brief Check transition from RDD to GROUND.
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
};

#endif