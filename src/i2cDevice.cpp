#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		    //Needed for I2C port

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

    bool acquire_i2c_(unsigned char address) {
        if (file_i2c() < 0) return false;
        static unsigned char prev_address{ 0 };
        if (prev_address == address) return true;
        bool success = (ioctl(file_i2c(), I2C_SLAVE, address) >= 0);
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

constexpr unsigned char SWRST = 0x06;
void i2cSoftwareReset() {
    WRAP(i2c_smbus_write_quick(file_i2c(), SWRST));
}

bool i2cDevice::acquire_i2c() {
    return acquire_i2c_(address);
}

void i2cDevice::write_quick(unsigned char value) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_quick(file_i2c(), value));
}

unsigned char i2cDevice::read_byte() {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_read_byte(file_i2c()));
    return static_cast<unsigned char>(ret);
}

void i2cDevice::write_byte(unsigned char value) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_byte(file_i2c(), value));
}

unsigned char i2cDevice::read_byte_data(unsigned char reg) {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_read_byte_data(file_i2c(), reg));
    std::cout << "Read value "
        << std::hex << ret
        << " from register "
        << reg << std::dec << std::endl;
    return static_cast<unsigned char>(ret);
}

bool i2cDevice::read_bit(unsigned char reg, unsigned char bitmask) {
    return (read_byte_data(reg) & bitmask) != 0;
}

void i2cDevice::write_byte_data(unsigned char reg, unsigned char value) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_byte_data(file_i2c(), reg, value));
    std::cout << "Wrote value "
        << std::hex << value
        << " to register "
        << reg << std::dec << std::endl;
    read_byte_data(reg);
}

void i2cDevice::write_bit(unsigned char reg, unsigned char bitmask, bool value) {
    if (!acquire_i2c()) return;
    unsigned char curr_value = read_byte_data(reg);
    unsigned char new_value = value ? (curr_value | bitmask) : (curr_value & ~bitmask);
    write_byte_data(reg, new_value);
}

uint16_t i2cDevice::read_word_data(unsigned char reg) {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_read_word_data(file_i2c(), reg));
    return static_cast<uint16_t>(ret);
}

void i2cDevice::write_word_data(unsigned char reg, uint16_t value) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_word_data(file_i2c(), reg, value));
}

uint16_t i2cDevice::process_call(unsigned char reg, uint16_t value) {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_process_call(file_i2c(), reg, value));
    return static_cast<uint16_t>(ret);
}

unsigned char i2cDevice::read_block_data(unsigned char reg, unsigned char* buffer) {
    if (!acquire_i2c()) return 0;
    WRAP(i2c_smbus_read_block_data(file_i2c(), reg, buffer));
    return static_cast<unsigned char>(ret);
}

void i2cDevice::write_block_data(unsigned char reg, unsigned char length, const unsigned char* buffer) {
    if (!acquire_i2c()) return;
    WRAP(i2c_smbus_write_block_data(file_i2c(), reg, length, buffer));
}
