#pragma once
#include <memory>
#include <array>
#include <atomic>

using value3d = std::array<double, 3>;

class ServoController;
class ADS1115;
class MPU9250;

class Hardware {
private:
    bool adcPotInput{ false };
    std::atomic_bool buttonL{ false };
    std::atomic_bool buttonR{ false };

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

    double getLightVoltage();
    double getPotentiometerVoltage();
    
    value3d getAcceleration();
    value3d getMagnetometer();
    double getTemperature();

    void buttonPressL();
    void buttonPressR();

    void calibratePan();
};