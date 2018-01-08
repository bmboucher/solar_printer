#pragma once
#include <PCA9685.hpp>

class ServoController : protected PCA9685 {
private:
    void init();

public:
    ServoController() : PCA9685() { init(); }
    ServoController(unsigned char address) : PCA9685(address) { init(); }

    void setPulseWidth
        (unsigned char servo, double pwm_phase, double pulse_ms);
    void setServoPosition
        (unsigned char servo, double pwm_phase, double position);
    double getResolution();
};