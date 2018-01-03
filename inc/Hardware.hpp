#pragma once

#include <ServoController.hpp>

class Hardware {
private:
    ServoController servoController;
public:
    Hardware() = default;

    void setMirrorPan(double deg);
    void setMirrorTilt(double deg);
};