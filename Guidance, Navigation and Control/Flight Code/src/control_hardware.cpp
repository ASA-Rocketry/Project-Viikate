#include "control_hardware.h"

ControlHardware::ControlHardware() {}

bool ControlHardware::Initialize() {
    // NOTE: Assumes servo pins are defined in constants.h
    canard_servo1_.attach(constants::kServo1Pin);
    canard_servo2_.attach(constants::kServo2Pin);
    canard_servo3_.attach(constants::kServo3Pin);
    canard_servo4_.attach(constants::kServo4Pin);

    initialisationAnimation();
    delay(5000);

    Serial.println("Control hardware initialized: Servos attached.");
    // Set all servos to neutral 90 degree position
    SetCanardAngle(constants::kServoNeutralAngle);

    return true;
}

void ControlHardware::initialisationAnimation() {
    // Example: Sweep from min to max and back
    canard_servo1_.write(constants::kServoMinAngle + constants::kServoTrim1);
    canard_servo2_.write(constants::kServoMinAngle + constants::kServoTrim2);
    canard_servo3_.write(constants::kServoMinAngle + constants::kServoTrim3);
    canard_servo4_.write(constants::kServoMinAngle + constants::kServoTrim4);
    delay(500);
    canard_servo1_.write(constants::kServoMaxAngle + constants::kServoTrim1);
    canard_servo2_.write(constants::kServoMaxAngle + constants::kServoTrim2);
    canard_servo3_.write(constants::kServoMaxAngle + constants::kServoTrim3);
    canard_servo4_.write(constants::kServoMaxAngle + constants::kServoTrim4);
    delay(500);
    canard_servo1_.write(
        constants::kServoNeutralAngle + constants::kServoTrim1
    );
    canard_servo2_.write(
        constants::kServoNeutralAngle + constants::kServoTrim2
    );
    canard_servo3_.write(
        constants::kServoNeutralAngle + constants::kServoTrim3
    );
    canard_servo4_.write(
        constants::kServoNeutralAngle + constants::kServoTrim4
    );
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
    canard_servo1_.write(servo_angle + constants::kServoTrim1);
    canard_servo2_.write(servo_angle + constants::kServoTrim2);
    canard_servo3_.write(servo_angle + constants::kServoTrim3);
    canard_servo4_.write(servo_angle + constants::kServoTrim4);
}
