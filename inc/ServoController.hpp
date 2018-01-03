#pragma once
#include <PCA9685.hpp>

class ServoController : protected PCA9685 {
private:
    void init();

public:
    ServoController() : PCA9685() { init(); }
    ServoController(__u8 address) : PCA9685(address) { init(); }

    void setServoPosition(__u8 servo, double pwm_phase, double position);
    double getResolution();
};