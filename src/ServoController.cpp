#include <ServoController.hpp>
#include <iostream>

constexpr double SERVO_PWM_FREQ = 50;
constexpr double MIN_PULSE_MS = 1.0;
constexpr double MAX_PULSE_MS = 2.0;

void ServoController::init() {
    setAutoIncrement();
    setOutputChangeOnAck();
    setTotemPoleOutputs();
    setPWMFreq(SERVO_PWM_FREQ);
    setAllPWMConstant(false);
    start();
}

void ServoController::setServoPosition
        (__u8 servo, double pwm_phase, double position) 
{
    if (position < 0 || position > 1) {
        std::cerr << "Invalid servo position " << position << std::endl;
    }
    const double pulse_width 
        = MIN_PULSE_MS + position * (MAX_PULSE_MS - MIN_PULSE_MS);
    const double pwm_width
        = 1000.0 / pwm_freq;
    const double duty_cycle = pulse_width / pwm_width;
    setPWM(servo, pwm_phase, duty_cycle);
}

double ServoController::getResolution() 
{
    const double freq_res = PCA9685::getResolution();
    const double pwm_width = 1000.0 / pwm_freq;
    const double ms_res = pwm_width * freq_res;
    return ms_res / (MAX_PULSE_MS - MIN_PULSE_MS);
}