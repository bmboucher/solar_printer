#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

#include <i2cDevice.hpp>
#include <iostream>
#include <iomanip>
#include <stdexcept>

using std::vector;

namespace {
    const char* i2c_filename = "/dev/i2c-1";
    int file_i2c() {
        static int file_i2c = open(i2c_filename, O_RDWR);
        if (file_i2c < 0) {
            std::cerr << "Failed to open the i2c bus" << std::endl;
        }
        return file_i2c;
    }

    bool acquire_i2c_(char address) {
        if (file_i2c() < 0) return false;
        static char prev_address{ 0 };
        if (prev_address == address) return true;
        bool success = (ioctl(file_i2c(), I2C_SLAVE, addr) >= 0);
        if (!success) {
            std::cerr << "Failed to acquire the i2c bus and "
                      << "communicate with device at "
                      << std::hex << address << std::dec << std::endl;
        } else {
            prev_address = address;
        }
        return success;
    }
}

#define WRAP(X) \
    int ret = X; \
    do { \
        if (ret < 0) { \
            throw new std::runtime_error(#X); \
            ret = 0; \
        } \
    } while (false)

bool i2cDevice::acquire_i2c() {
    return acquire_i2c_(address);
}

void i2cDevice::write_quick(char value) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_quick(file_i2c(), value));
}

char i2cDevice::read_byte() {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_read_byte(file_i2c()));
    return static_cast<char>(ret);
}

void i2cDevice::write_byte(char value) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_byte(file_i2c(), value));
}

char i2cDevice::read_byte_data(char reg) {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_read_byte_data(file_i2c(), reg));
    return static_cast<char>(ret);
}

void i2cDevice::write_byte_data(char reg, char value) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_byte_data(file_i2c(), reg, value));
}

uint16_t i2cDevice::read_word_data(char reg) {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_read_word_data(file_i2c(), reg));
    return static_cast<uint16_t>(ret);
}

void i2cDevice::write_word_data(char reg, uint16_t value) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_word_data(file_i2c(), reg, value));
}

uint16_t i2cDevice::process_call(char reg, uint16_t value) {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_process_call(file_i2c(), reg, value));
    return static_cast<uint16_t>(ret);
}

char i2cDevice::read_block_data(char reg, char* buffer) {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_read_block_data(file_i2c(), reg, buffer));
    return static_cast<char>(ret);
}

void i2cDevice::write_block_data(char reg, char length, char* buffer) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_block_data(file_i2c(), reg, length, buffer));
}