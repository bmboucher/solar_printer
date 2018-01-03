#pragma once

#include <vector>
#include <cstdint>

class i2cDevice {
protected:
    const __u8 address;

public:
    i2cDevice(__u8 address) : address(address) {}

    bool acquire_i2c();
    void write_quick(__u8 value);
    __u8 read_byte();
    void write_byte(__u8 value);
    __u8 read_byte_data(__u8 reg);
    bool read_bit(__u8 reg, __u8 bitmask);
    void write_byte_data(__u8 reg, __u8 value);
    void write_bit(__u8 reg, __u8 bitmask, bool value);
    uint16_t read_word_data(__u8 reg);
    void write_word_data(__u8 reg, uint16_t value);
    uint16_t process_call(__u8 reg, uint16_t value);
    __u8 read_block_data(__u8 reg, __u8* buffer);
    void write_block_data(__u8 reg, __u8 length, const __u8* buffer);
};