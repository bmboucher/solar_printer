#include <ServoController.hpp>
#include <i2c.hpp>
#include <unistd.h>

constexpr char DEFAULT_ADDR = 0x40;
constexpr char MODE1 = 0x00;
constexpr char MODE2 = 0x01;
constexpr char SUBADR1 = 0x02;
constexpr char SUBADR2 = 0x03;
constexpr char SUBADR3 = 0x04;
constexpr char PRESCALE = 0xFE;
constexpr char LED0_ON_L = 0x06;
constexpr char LED0_ON_H = 0x07;
constexpr char LED0_OFF_L = 0x08;
constexpr char LED0_OFF_H = 0x09;
constexpr char ALL_LED_ON_L = 0xFA;
constexpr char ALL_LED_ON_H = 0xFB;
constexpr char ALL_LED_OFF_L = 0xFC;
constexpr char ALL_LED_OFF_H = 0xFD;
constexpr char RESTART = 0x80;
constexpr char SLEEP = 0x10;
constexpr char ALLCALL = 0x01;
constexpr char INVRT = 0x10;
constexpr char OUTDRV = 0x04;

constexpr double SERVO_FREQ = 60;

class ServoController::Implementation {
private:
    char address;

    char readByte(char reg) { 
        writeI2C(address, { reg });
        return readI2Cbyte(address);
    }
    void writeRegister(char reg, char value) {
        writeI2C(address, { reg, value });
    }
public:
    Implementation(char address) : address(address) {
        writeRegister(ALL_LED_ON_L, 0);
        writeRegister(ALL_LED_ON_H, 0);
        writeRegister(ALL_LED_OFF_L, 0);
        writeRegister(ALL_LED_OFF_H, 0);
        writeRegister(MODE2, OUTDRV);
        writeRegister(MODE1, ALLCALL);
        usleep(5000);
        char mode1 = readI2Cbyte
    }
};

ServoController::ServoController() : ServoController(DEFAULT_ADDR) {}
ServoController::ServoController(char address)
    : pImpl(new ServoController::Implementation(address)) {}
ServoController::~ServoController() = default;

void ServoController::setServoPosition(char servo, double position) {

}
