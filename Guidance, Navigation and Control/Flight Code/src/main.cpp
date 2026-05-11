/*
 * src/main.cpp
 *
 * This is the Arduino application entry point for the flight control system.
 * It creates the core objects for logging, flight state management, sensor
 * acquisition, control actuation, and executes a simple setpoint test loop.
 *
 * The main loop reads live flight data, records telemetry, performs PID control
 * on the current orientation setpoint, and sends telemetry to an auxiliary serial
 * port for monitoring.
 */

#include "Arduino.h"
#include "constants.h"
#include "control.h"
#include "control_hardware.h"
#include "data_logger.h"
#include "flight_data.h"
#include "sensors.h"
#include "state_machine.h"

// Global subsystem instances used throughout the flight control loop.
DataLogger data_logger;
Sensors sensors(data_logger);
ControlHardware control_hardware;
Control control;
StateMachine state_machine(data_logger, sensors, control);

// Simulation state variables for TEST_STATE_MACHINE_MODE
bool simulation_started = false;
bool coastManeuverStarted = false;
unsigned long coastStartTime = 0;

// Last computed control error used for status reporting and LED state.
float error;

#ifdef TEST_PID_AND_CALIBRATION_MODE
static unsigned long lastSwitchTime = 0;
static constexpr unsigned long interval = 5000;
#endif
// Predefined setpoints for PID testing (0° and 90° roll).
// Using constexpr array for fixed setpoints known at compile time.
// This is more efficient for embedded systems than dynamic structures like std::vector.
constexpr float setpoints[] = {0.0f, 90.0f};
constexpr size_t numSetpoints = sizeof(setpoints) / sizeof(setpoints[0]);

static constexpr size_t setpointIndexInitial = 0;
static size_t setpointIndex = setpointIndexInitial;

// Binary-packed telemetry packet structure sent over Serial8.
struct TelemetryData {
    float altitude;
    float verticalVelocity;
    float accZ;
    float rotZ;
    float error;
    uint32_t timestamp;
} __attribute__((packed));

/**
 * @brief Send a packed telemetry packet to a serial port.
 *
 * This function converts the selected FlightData values and the current
 * control error into a compact binary packet for external logging or analysis.
 * It uses Serial8 as the dedicated telemetry transport path.
 */
#if defined(TEST_STATE_MACHINE_MODE) || defined(TEST_PID_AND_CALIBRATION_MODE)
void sendToSerial(Print& serial, const FlightData& data, const Control& control) {
    TelemetryData telem;
    telem.altitude = data.altitude;
    telem.verticalVelocity = data.verticalVelocity;
    telem.accZ = data.accZ;
    telem.rotZ = data.oriZ;
    telem.error = control.get_error();
    telem.timestamp = micros();

    serial.write((uint8_t *)&telem, sizeof(telem));
}
#endif

void setup() {
    #if defined(TEST_STATE_MACHINE_MODE) || defined(TEST_PID_AND_CALIBRATION_MODE)
        // Initialize the primary debug serial port.
        Serial.begin(9600);
        #ifdef TEST_STATE_MACHINE_MODE
            Serial8.begin(9600);
        #endif
        delay(1000);  // Give serial time to initialize
    #endif

    #ifdef TEST_STATE_MACHINE_MODE
        Serial.println("\n\n=== TEST STATE MACHINE MODE STARTING ===");

    #elif TEST_PID_AND_CALIBRATION_MODE
        Serial.println("\n\n=== TEST PID AND CALIBRATION MODE STARTING ===");
    #endif

    // Initialize core subsystems in a safe order.
    // Data logger first so that any failures during sensors/control init can be recorded.
    if (!data_logger.Initialize()) {
        #ifdef TEST_STATE_MACHINE_MODE
            Serial.println("Failed to initialize DataLogger!");
        #elif TEST_PID_AND_CALIBRATION_MODE
            Serial.println("Failed to initialize DataLogger!");
        #endif
        return;
    }
    if (!sensors.Initialize()) {
        #ifdef TEST_STATE_MACHINE_MODE
            Serial.println("Failed to initialize Sensors!");
        #elif TEST_PID_AND_CALIBRATION_MODE
            Serial.println("Failed to initialize Sensors!");
        #endif
        return;
    }
    if (!control.Initialize()) {
        #ifdef TEST_STATE_MACHINE_MODE
            Serial.println("Failed to initialize Control!");
        #elif TEST_PID_AND_CALIBRATION_MODE
            Serial.println("Failed to initialize Control!");
        #endif
        return;
    }
    #ifdef TEST_STATE_MACHINE_MODE
        readSimulatedData();
    #endif

    #ifdef TEST_STATE_MACHINE_MODE
        Serial.println("\n\n=== TEST STATE MACHINE MODE ===");
    #elif TEST_PID_AND_CALIBRATION_MODE
        Serial.println("\n\n=== TEST PID AND CALIBRATION MODE ===");
    #endif

    pinMode(constants::kLEDPin, OUTPUT);
    pinMode(constants::kRbfPin, INPUT_PULLUP);  // Hardware interlock pin

    data_logger.LogEvent(LogType::kInfo, "SETUP COMPLETE");
}

/**
 * @brief Main Arduino loop that executes the flight control cycle.
 *
 * Each iteration reads the latest flight sensor values, stores them in the data
 * logger, updates the PID controller on the current test setpoint, and sends
 * out telemetry over both serial outputs.
 */
void loop() {
    // Read real hardware data
    FlightData live_data = sensors.ReadFlightData();
    FlightData state_input_data = live_data;
    #ifdef TEST_PID_AND_CALIBRATION_MODE
        // In PID test mode, we use live sensor data for state machine transitions,
        // but the control loop will be tested against the predefined setpoints.
            // Acquire the latest sensor and attitude measurements.
        data_logger.LogFlightData(state_input_data);

        unsigned long currentTime = millis();

        // Advance the test setpoint every fixed interval to verify controller response.
        if (currentTime - lastSwitchTime >= interval) {
            lastSwitchTime = currentTime;
            setpointIndex = (setpointIndex + 1) % numSetpoints;
        }
        
        // Run the PID controller against the current orientation setpoint.
        control.PID(
            setpoints[setpointIndex],
            state_input_data.oriZ
        );

        // Print the current sensor and control state for local debugging.
        Serial.print("Altitude: ");
        Serial.println(state_input_data.altitude);
        Serial.print("Vertical Velocity: ");
        Serial.println(state_input_data.verticalVelocity);
        Serial.print("AccelZ: ");
        Serial.println(state_input_data.accZ);
        Serial.print("RotZ: ");
        Serial.println(state_input_data.rotZ);
        Serial.print("AccelMagnitude: ");
        Serial.println(state_input_data.accelMagnitude);
        Serial.print("RBF Removed: ");
        Serial.println(state_input_data.rbfRemoved);
        Serial.print("Acc (x, y, z): ");
        Serial.print(state_input_data.accX);
        Serial.print(", ");
        Serial.print(state_input_data.accY);
        Serial.print(", ");
        Serial.println(state_input_data.accZ);
        Serial.print("Gyro Angular Rate (x, y, z): ");
        Serial.print(state_input_data.rotX);
        Serial.print(", ");
        Serial.print(state_input_data.rotY);
        Serial.print(", ");
        Serial.println(state_input_data.rotZ);
        Serial.print("Orientation (x, y, z): ");
        Serial.print(state_input_data.oriX);
        Serial.print(", ");
        Serial.print(state_input_data.oriY);
        Serial.print(", ");
        Serial.println(state_input_data.oriZ);
        Serial.print("Mag: ");
        Serial.print(state_input_data.magX);
        Serial.print(", ");
        Serial.print(state_input_data.magY);
        Serial.print(", ");
        Serial.println(state_input_data.magZ);
        Serial.print("Heading: ");
        Serial.println(state_input_data.heading);
        Serial.println("--------------------");

        // Display the current control error and active setpoint.
        Serial.println("Control error:");
        error = control.get_error();
        Serial.println(error);
        Serial.print("Current setpoint: ");
        Serial.println(setpoints[setpointIndex]);

        // Use the onboard LED to indicate whether the controller is within tolerance.
        if (abs(error) < 5.0f) {
            digitalWrite(constants::kLEDPin, HIGH);
        } else {
            digitalWrite(constants::kLEDPin, LOW);
            delay(50);
        }

        // Send the compact telemetry packet to the secondary serial port.
        sendToSerial(Serial8, state_input_data, control);
    #else
        
        /* 
        ### kCalibration ### 
        In this state the setup takes care of the initialization of the subsystems and loading of the simulated data. 
        */

        /* 
        ### kIdle ### 
        The setup and calibration is done, but the launch interlock is still engaged. 
        The state machine should not transition out of this state until the RBF pin is pulled.
        */ 

        #ifdef TEST_STATE_MACHINE_MODE
            // Start simulation AFTER entering launchpad state
            if (state_machine.GetState() == State::kLaunchpad &&
                !simulation_started) {

                simulation_started = true;

                Serial.println("Simulation started");
            }
        #endif
        /* 
        ### kLaunchpad ### 
        The launch interlock has been removed, but the rocket is still on the launchpad.
        The state machine should transition to liftoff once the vertical acceleration exceeds the desired threshold.
        */ 
        
        /*
        ### kLiftoff ###
        The rocket has left the launchpad and is accelerating upwards.
        */

        #ifdef TEST_STATE_MACHINE_MODE
            if (simulation_started) {

                FlightData simulated_data = getSimulatedFlightData();

                // Hardware-in-the-loop inputs
                simulated_data.oriZ = live_data.oriZ;
                simulated_data.rbfRemoved = live_data.rbfRemoved;

                state_input_data = simulated_data;
            }
        #endif
        // Update state machine
        State current_state = state_machine.Update(state_input_data);

        unsigned long currentTime = millis();

        // Control logic
        switch (current_state) {
            case State::kLiftoff: {

            // Force neutral during liftoff
            control.SetCanardAngle(0.0f);

            // Reset coast logic so it starts fresh later
            coastManeuverStarted = false;

            break;
            }
            /* 
            ### kCoast ### 
            At the beginning of coast phase, perform the roll maneuver to 90 deg and back.
            */
            case State::kCoast: {

                // Start maneuver once when entering coast
                if (!coastManeuverStarted) {
                    coastManeuverStarted = true;
                    coastStartTime = currentTime;
                }

                float coastSetpoint;

                // First 2 seconds: roll to 90°
                if (currentTime - coastStartTime < 2000) {
                    coastSetpoint = 90.0f;
                }
                // After 2 seconds: return to neutral
                else {
                    coastSetpoint = 0.0f;
                }

                control.PID(
                    coastSetpoint,
                    state_input_data.oriZ
                );

                break;
            }
            /* 
            ### kApogee ### 
            The PID control can be turned of before recovery device deployment.
            */

            case State::kApogee: {
                control.SetCanardAngle(0.0f);
                break;
            }
            /* 
            ### kRDD ### 
            In the current implementation, the RDD is passive.
            */

            case State::kRDD: {
                control.SetCanardAngle(0.0f);
                break;
            }
            /*
                ### kGround ###
                The vehicle has hit the ground, and data recording can be stoped
                after a short delay.
                */

            case State::kGround: {

                control.SetCanardAngle(0.0f);
                break;
            }
            default: {
                break;
            }
        }

        #ifdef TEST_STATE_MACHINE_MODE
            // Debug output
            Serial.print("Current state: ");
            Serial.println(StateToString(current_state));

            Serial.print("Time (s): ");
            Serial.println(state_input_data.timeMs / 1000.0f);

            Serial.print("Altitude (m): ");
            Serial.println(state_input_data.altitude);

            Serial.print("Vertical velocity (m/s): ");
            Serial.println(state_input_data.verticalVelocity);

            Serial.print("Vertical acceleration (m/s²): ");
            Serial.println(state_input_data.accZ);

            Serial.print("Total acceleration (m/s²): ");
            Serial.println(state_input_data.accelMagnitude);

            Serial.print("RBF Removed: ");
            Serial.println(state_input_data.rbfRemoved);

            Serial.print("Orientation Z (live): ");
            Serial.println(state_input_data.oriZ);

            Serial.print("Current roll setpoint: ");
            Serial.println(setpoints[setpointIndex]);

            Serial.println("--------------------");

            // Uncomment this for slower state change reading.
            //delay(500);
        #endif
    #endif
}
