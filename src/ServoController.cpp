#include <ServoController.hpp>
#include <iostream>

constexpr double SERVO_PWM_FREQ = 50;
constexpr double MIN_PULSE_MS = 0.5;
constexpr double MAX_PULSE_MS = 2.5;

void ServoController::init() {
    setAutoIncrement();
    setOutputChangeOnAck();
    setTotemPoleOutputs();
    setPWMFreq(SERVO_PWM_FREQ);
    setAllPWMConstant(false);
    start();
}

void ServoController::setPulseWidth
        (unsigned char servo, double pwm_phase, double pulse_ms) 
{
    PCA9685::setPulseWidth(servo, pwm_phase, pulse_ms);
}

void ServoController::setServoPosition
        (unsigned char servo, double pwm_phase, double position) 
{
    if (position < 0 || position > 1) {
        std::cerr << "Invalid servo position " << position << std::endl;
    }
    const double pulse_ms
        = MIN_PULSE_MS + position * (MAX_PULSE_MS - MIN_PULSE_MS);
    setPulseWidth(servo, pwm_phase, pulse_ms);
}

double ServoController::getResolution() 
{
    const double freq_res = PCA9685::getResolution();
    const double pwm_width = 1000.0 / getPWMFreq();
    const double ms_res = pwm_width * freq_res;
    return ms_res / (MAX_PULSE_MS - MIN_PULSE_MS);
}

double ServoController::getPulseWidth(unsigned char servo) {
    return PCA9685::getPulseWidth(servo);
}

double ServoController::getServoPosition(unsigned char servo) {
    const double pulse_ms = getPulseWidth(servo);
    return (pulse_ms - MIN_PULSE_MS) / (MAX_PULSE_MS - MIN_PULSE_MS);
}