#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		    //Needed for I2C port

#include <i2cDevice.hpp>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <bitset>

using std::vector;
using std::bitset;

namespace {
    const char* i2c_filename = "/dev/i2c-1";
    int file_i2c() {
        static int file_i2c = open(i2c_filename, O_RDWR);
        if (file_i2c < 0) {
            std::cerr << "Failed to open the i2c bus" << std::endl;
        }
        return file_i2c;
    }

    bool address_set{ false };
    unsigned char curr_address{ 0 };
    bool acquire_i2c(unsigned char address) {
        if (file_i2c() < 0) return false;
        if (address_set && curr_address == address) return true;
        bool success = (ioctl(file_i2c(), I2C_SLAVE, address) >= 0);
        if (!success) {
            std::cerr << "Failed to acquire the i2c bus and "
                      << "communicate with device at "
                      << std::hex << address << std::dec << std::endl;
        } else {
            address_set = true;
            curr_address = address;
        }
        return success;
    }

    bool logging{ false };
    void i2cLog(bool write) {
        if (!logging) return;
        std::cout << bitset<7>(curr_address) << ' '
            << (write ? '1' : '0') << ' '
            << "-------- --------" << std::endl;
    }
    void i2cLog(bool write, unsigned char value) {
        if (!logging) return;
        std::cout << bitset<7>(curr_address) << ' '
            << (write ? '1' : '0') << ' '
            << "-------- "
            << bitset<8>(value) << std::endl;
    }
    void i2cLog(bool write, unsigned char reg, unsigned char value) {
        if (!logging) return;
        std::cout << bitset<7>(curr_address) << ' '
                  << (write ? '1' : '0') << ' '
                  << bitset<8>(reg) << ' '
                  << bitset<8>(value) << std::endl;
    }
    void i2cLog(bool write, unsigned char reg, uint16_t value) {
        if (!logging) return;
        std::cout << bitset<7>(curr_address) << ' '
            << (write ? '1' : '0') << ' '
            << bitset<8>(reg) << ' '
            << bitset<16>(value) << std::endl;
    }
    void i2cLog(unsigned char reg, bool write,
                unsigned char length, const unsigned char* values) {
        if (!logging) return;
        i2cLog(reg, write, values[0]);
        for (unsigned char i = 1; i < length; i++)
            i2cLog(write, values[i]);
    }
}

void i2c::setLogging(bool value) { logging = value; }

#define WRAP(X) \
    int ret = X; \
    do { \
        if (ret < 0) { \
            std::cerr << "Error during " << #X << std::endl; \
            ret = 0; \
        } \
    } while (false)


// Quick write (just the R/W bit)
void i2c::write_quick(unsigned char addr, bool rw_bit) {
    if (!acquire_i2c(addr)) return;
    unsigned char value = rw_bit ? 1 : 0;
    WRAP(i2c_smbus_write_quick(file_i2c(), value));
    i2cLog(rw_bit);
}
void i2cDevice::write_quick(bool rw_bit) {
    i2c::write_quick(address, rw_bit);
}

// Read byte (no register)
unsigned char i2c::read_byte(unsigned char addr) {
    if (!acquire_i2c(addr)) return 0;
    WRAP(i2c_smbus_read_byte(file_i2c()));
    unsigned char byte = static_cast<unsigned char>(ret);
    i2cLog(false, byte); return byte;
}
unsigned char i2cDevice::read_byte() {
    return i2c::read_byte(address);
}

// Write byte (no register)
void i2c::write_byte(unsigned char addr, unsigned char value) {
    if (!acquire_i2c(addr)) return;
    WRAP(i2c_smbus_write_byte(file_i2c(), value));
    i2cLog(true, value);
}
void i2cDevice::write_byte(unsigned char value) {
    i2c::write_byte(address, value);
}

// Read byte from register
unsigned char i2c::read_byte_data(unsigned char addr, unsigned char reg) {
    if (!acquire_i2c(addr)) return 0;
    WRAP(i2c_smbus_read_byte_data(file_i2c(), reg));
    unsigned char byte = static_cast<unsigned char>(ret);
    i2cLog(false, reg, byte); return byte;
}
unsigned char i2cDevice::read_byte_data(unsigned char reg) {
    return i2c::read_byte_data(address, reg);
}

// Read bit
bool i2c::read_bit
    (unsigned char addr, unsigned char reg, unsigned char bitmask) {
    return (i2c::read_byte_data(addr, reg) & bitmask) != 0;
}
bool i2cDevice::read_bit(unsigned char reg, unsigned char bitmask) {
    i2c::read_bit(address, reg, bitmask);
}

// Write byte to register
void i2c::write_byte_data
        (unsigned char addr, unsigned char reg, unsigned char value) 
{
    if (!acquire_i2c(addr)) return;
    WRAP(i2c_smbus_write_byte_data(file_i2c(), reg, value));
    i2cLog(true, reg, value);
}
void i2cDevice::write_byte_data(unsigned char reg, unsigned char value) {
    i2c::write_byte_data(address, reg, value);
}

// Write bit
void i2c::write_bit
    (unsigned char addr, unsigned char reg, unsigned char bitmask, bool value) 
{
    if (!acquire_i2c(addr)) return;
    unsigned char curr_value = i2c::read_byte_data(addr, reg);
    unsigned char new_value 
        = value ? (curr_value | bitmask) : (curr_value & ~bitmask);
    i2c::write_byte_data(addr, reg, new_value);
}
void i2cDevice::write_bit
    (unsigned char reg, unsigned char bitmask, bool value) {
    i2c::write_bit(address, reg, bitmask, value);
}

// Read word from register
uint16_t i2c::read_word_data(unsigned char addr, unsigned char reg) {
    if (!acquire_i2c(addr)) return 0;
    WRAP(i2c_smbus_read_word_data(file_i2c(), reg));
    const uint16_t word = static_cast<uint16_t>(ret);
    i2cLog(false, reg, word); return word;
}
uint16_t i2cDevice::read_word_data(unsigned char reg) {
    return i2c::read_word_data(address, reg);
}

// Write word to register
void i2c::write_word_data
    (unsigned char addr, unsigned char reg, uint16_t value) 
{
    if (!acquire_i2c(addr)) return;
    WRAP(i2c_smbus_write_word_data(file_i2c(), reg, value));
    i2cLog(true, reg, value);
}
void i2cDevice::write_word_data(unsigned char reg, uint16_t value) {
    i2c::write_word_data(address, reg, value);
}

// Write word to register, then read register back out
uint16_t i2c::process_call
    (unsigned char addr, unsigned char reg, uint16_t value) 
{
    if (!acquire_i2c(addr)) return 0;
    WRAP(i2c_smbus_process_call(file_i2c(), reg, value));
    const uint16_t return_word = static_cast<uint16_t>(ret);
    i2cLog(true, reg, value);
    i2cLog(false, reg, return_word);
    return return_word;
}
uint16_t i2cDevice::process_call(unsigned char reg, uint16_t value) {
    return i2c::process_call(address, reg, value);
}

// Read multiple registers
unsigned char i2c::read_block_data
        (unsigned char addr, unsigned char reg, unsigned char* buffer) 
{
    if (!acquire_i2c(addr)) return 0;
    WRAP(i2c_smbus_read_block_data(file_i2c(), reg, buffer));
    unsigned char n_bytes = static_cast<unsigned char>(ret);
    i2cLog(reg, false, n_bytes, buffer); return n_bytes;
}
unsigned char i2cDevice::read_block_data
    (unsigned char reg, unsigned char* buffer) 
{
    return i2c::read_block_data(address, reg, buffer);
}

// Write multiple registers
void i2c::write_block_data
        (unsigned char addr, unsigned char reg, unsigned char length, const unsigned char* buffer) 
{
    if (!acquire_i2c(addr)) return;
    WRAP(i2c_smbus_write_block_data(file_i2c(), reg, length, buffer));
    i2cLog(reg, true, length, buffer);
}
void i2cDevice::write_block_data
    (unsigned char reg, unsigned char length, const unsigned char* buffer) 
{
    i2c::write_block_data(address, reg, length, buffer);
}

constexpr unsigned char GEN_ADDR = 0x00;
constexpr unsigned char SWRST = 0x06;
void i2c::softwareReset() {
    i2c::write_byte(GEN_ADDR, SWRST);
}
