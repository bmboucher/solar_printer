#pragma once

#include <vector>
#include <cstdint>

namespace i2c {

void write_quick(unsigned char addr, bool rw_bit);
unsigned char read_byte(unsigned char addr);
void write_byte(unsigned char addr, unsigned char value);
unsigned char read_byte_data(unsigned char addr, unsigned char reg);
bool read_bit(unsigned char addr, unsigned char reg, unsigned char bitmask);
void write_byte_data
    (unsigned char addr, unsigned char reg, unsigned char value);
void write_bit
    (unsigned char addr, unsigned char reg, unsigned char bitmask, bool value);
uint16_t read_word_data
    (unsigned char addr, unsigned char reg);
void write_word_data
    (unsigned char addr, unsigned char reg, uint16_t value);
uint16_t process_call
    (unsigned char addr, unsigned char reg, uint16_t value);
unsigned char read_block_data
    (unsigned char addr, unsigned char reg, unsigned char* buffer);
void write_block_data
    (unsigned char addr, unsigned char reg, 
           unsigned char length, const unsigned char* buffer);

void setLogging(bool value);
void softwareReset();

} // namespace i2c

class i2cDevice {
protected:
    const unsigned char address;

public:
    i2cDevice(unsigned char address) : address(address) {}

    void write_quick(bool rw_bit);
    unsigned char read_byte();
    void write_byte(unsigned char value);
    unsigned char read_byte_data(unsigned char reg);
    bool read_bit(unsigned char reg, unsigned char bitmask);
    void write_byte_data(unsigned char reg, unsigned char value);
    void write_bit(unsigned char reg, unsigned char bitmask, bool value);
    uint16_t read_word_data(unsigned char reg);
    void write_word_data(unsigned char reg, uint16_t value);
    uint16_t process_call(unsigned char reg, uint16_t value);
    unsigned char read_block_data(unsigned char reg, unsigned char* buffer);
    void write_block_data(unsigned char reg, unsigned char length, const unsigned char* buffer);
};