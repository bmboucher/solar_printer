#pragma once

#include <vector>
#include <cstdint>

class i2cDevice {
protected:
    const char address;

    bool acquire_i2c();
    void write_quick(char value);
    char read_byte();
    void write_byte(char value);
    char read_byte_data(char reg);
    void write_byte_data(char reg, char value);
    uint16_t read_word_data(char reg);
    void write_word_data(char reg, uint16_t value);
    uint16_t process_call(char reg, uint16_t value);
    char read_block_data(char reg, char* buffer);
    void write_block_data(char reg, char length, char* buffer);

public:
    i2c_device(char address) : address(address) {}
};