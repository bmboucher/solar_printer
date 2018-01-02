#pragma once
#include <memory>

class ServoController {
private:
    class Implementation;
    std::unique_ptr<Implementation> pImpl;
public:
    ServoController();
    ServoController(char address);
    ~ServoController();
    void setServoPosition(char servo, double position);
};