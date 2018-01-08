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
    const double pwm_width = 1000.0 / pwm_freq;
    if (pulse_ms < 0 || pulse_ms > pwm_width) {
        std::cerr << "Invalid pulse width " << pulse_ms << " ms" << std::endl;
    }
    const double duty_cycle = pulse_ms / pwm_width;
    setPWM(servo, pwm_phase, duty_cycle);
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
    const double pwm_width = 1000.0 / pwm_freq;
    const double ms_res = pwm_width * freq_res;
    return ms_res / (MAX_PULSE_MS - MIN_PULSE_MS);
}
