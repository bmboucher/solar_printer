#pragma once
#include <memory>

class ServoController;
class ADS1115;

class Hardware {
private:
    std::unique_ptr<ServoController> servoController;
    std::unique_ptr<ADS1115> adc;
public:
    Hardware();
    Hardware(const Hardware& rhs) = delete;
    Hardware(Hardware&& rhs) = delete;
    ~Hardware();

    void setMirrorPan(double deg);
    void setMirrorTilt(double deg);
    double getVoltage();
};