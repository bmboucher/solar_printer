#pragma once
#include <memory>

class ServoController;

class Hardware {
private:
    std::unique_ptr<ServoController> servoController{ nullptr };
public:
    Hardware();
    Hardware(const Hardware& rhs) = delete;
    Hardware(Hardware&& rhs) = delete;
    ~Hardware();

    void setMirrorPan(double deg);
    void setMirrorTilt(double deg);
};