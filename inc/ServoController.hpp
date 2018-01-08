#pragma once
#include <PCA9685.hpp>

class ServoController : protected PCA9685 {
private:
    void init();

public:
    ServoController() : PCA9685() { init(); }
    ServoController(unsigned char address, unsigned char oePin) 
        : PCA9685(address, oePin) { init(); }

    void setPulseWidth
        (unsigned char servo, double pwm_phase, double pulse_ms);
    void setServoPosition
        (unsigned char servo, double pwm_phase, double position);
    double getResolution();
    double getPulseWidth(unsigned char servo);
    double getServoPosition(unsigned char servo);
};