#pragma once
#include <memory>
#include <array>

using value3d = std::array<double, 3>;

class ServoController;
class ADS1115;
class MPU9250;

class Hardware {
private:
    std::unique_ptr<ServoController> servoController;
    std::unique_ptr<ADS1115> adc;
    std::unique_ptr<MPU9250> mpu;
public:
    Hardware();
    Hardware(const Hardware& rhs) = delete;
    Hardware(Hardware&& rhs) = delete;
    ~Hardware();

    void setMirrorPan(double deg);
    void setMirrorTilt(double deg);
    double getVoltage();
    
    value3d getAcceleration();
    value3d getGyroscope();
    value3d getMagnetometer();
    double getTemperature();
};