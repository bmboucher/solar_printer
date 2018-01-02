#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

#include <ServoController.hpp>

const char default_address = 0x40;

class ServoController::Implementation {
private:
    char address;
public:
    Implementation(char address) : address(address) { }
};

ServoController::ServoController() : ServoController(default_address) {}
ServoController::ServoController(char address)
    : pImpl(new ServoController::Implementation(address)) {}
ServoController::~ServoController() = default;

void ServoController::setServoPosition(char servo, double position) {

}
