#include <Hardware.hpp>
#include <i2c.hpp>
#include <iostream>

#include <ServoController.hpp>

constexpr unsigned char PAN_SERVO = 0;
constexpr double PAN_SERVO_PHASE = 0;
constexpr unsigned char TILT_SERVO = 1;
constexpr double TILT_SERVO_PHASE = 0.5;

Hardware::Hardware() {
    i2c::softwareReset();
    servoController.reset(new ServoController());
}

~Hardware::Hardware() = default;

void Hardware::setMirrorPan(double deg) {
    if (!servoController) return;
    if (deg < -90 || deg > 90) {
        std::cerr << "Invalid pan angle " << deg << std::endl; return;
    }
    const double pos = (deg + 90) / 180;
    servoController->setServoPosition(PAN_SERVO, PAN_SERVO_PHASE, pos);
}

void Hardware::setMirrorTilt(double deg) {
    if (!servoController) return;
    if (deg < 0 || deg > 90) {
        std::cerr << "Invalid tilt angle " << deg << std::endl; return;
    }
    const double pos = (deg + 90) / 180;
    servoController->setServoPosition(TILT_SERVO, TILT_SERVO_PHASE, pos);
}