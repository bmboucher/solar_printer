#pragma once

class GPIOPin {
public:
    enum class Direction {
        INPUT,
        OUTPUT
    };
    GPIOPin(unsigned char pin, Direction dir = Direction::OUTPUT);
    GPIOPin(const GPIOPin& rhs) = delete;
    GPIOPin(GPIOPin&& rhs) = delete;
    ~GPIOPin();

    void setDirection(Direction dir);
    void setValue(bool value);
    bool getValue();

private:
    const unsigned char pin;
    Direction dir;
    bool value;
};