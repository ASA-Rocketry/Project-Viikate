#include "control_hardware.h"
#include "debug_prints.h"

ControlHardware::ControlHardware() {}

bool ControlHardware::Initialize() {
    // NOTE: Assumes servo pins are defined in constants.h
    servo1_.attach(constants::kServo1Pin);
    servo2_.attach(constants::kServo2Pin);
    servo3_.attach(constants::kServo3Pin);
    servo4_.attach(constants::kServo4Pin);

    initialisationAnimation();

    DEBUG_PRINTLN("Control hardware initialized!");

    return true;
}

void ControlHardware::initialisationAnimation() {
    SetCanardAngle(constants::kMinControlAngle);
    delay(500);

    SetCanardAngle(constants::kMaxControlAngle);
    delay(500);

    SetCanardAngle(constants::kNeutralControlAngle); // Neutral control angle defined in constants.h
    delay(500);
}
void ControlHardware::SetCanardAngle(float pid_angle_degrees) {
    // Clamp PID output to valid operating range from constants
    float clamped_pid = constrain(
        -pid_angle_degrees,
        constants::kPidMinAngle,
        constants::kPidMaxAngle
    );

    // Map from PID range to servo command range using constants
    float servo_angle =
        map(clamped_pid,
            constants::kPidMinAngle,
            constants::kPidMaxAngle,
            constants::kServoMinAngle,
            constants::kServoMaxAngle);

    // Apply the calculated angle to all four canard servos
    servo1_.write(servo_angle + constants::kServoTrim1);
    servo2_.write(servo_angle + constants::kServoTrim2);
    servo3_.write(servo_angle + constants::kServoTrim3);
    servo4_.write(servo_angle + constants::kServoTrim4);
}
