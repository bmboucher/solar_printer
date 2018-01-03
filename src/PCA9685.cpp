#include <PCA9685.hpp>

#include <unistd.h>
#include <cmath>
#include <iostream>
#include <vector>

using std::vector;

constexpr __u8 DEFAULT_ADDRESS = 0x40;

// Register addresses
constexpr __u8 MODE1        = 0x00;
constexpr __u8 MODE2        = 0x01;
constexpr __u8 SUBADR1      = 0x02;
constexpr __u8 SUBADR2      = 0x03;
constexpr __u8 SUBADR3      = 0x04;
constexpr __u8 ALLCALLADR   = 0x05;
constexpr __u8 LED0_ON_L    = 0x06;
    // Registers LEDi_ON_L, LEDi_ON_H, LEDi_OFF_L, LEDi_OFF_H etc...
constexpr __u8 ALL_LED_ON_L = 0xFA;
    // Registers ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H
constexpr __u8 PRE_SCALE    = 0xFE;

// Default software addresses
constexpr __u8 DEFAULT_SUBADDR1 = 0x71;
constexpr __u8 DEFAULT_SUBADDR2 = 0x72;
constexpr __u8 DEFAULT_SUBADDR3 = 0x74;
constexpr __u8 DEFAULT_ALLCALLADDR = 0x70;

// Mode 1 Bits
constexpr __u8 RESTART = 0x80;
constexpr __u8 EXTCLK = 0x40;
constexpr __u8 AI = 0x20;
constexpr __u8 SLEEP = 0x10;
constexpr __u8 SUB1 = 0x08;
constexpr __u8 SUB2 = 0x04;
constexpr __u8 SUB3 = 0x02;
constexpr __u8 ALLCALL = 0x01;

// Mode 2 Bits
constexpr __u8 INVRT = 0x10;
constexpr __u8 OCH = 0x08;
constexpr __u8 OUTDRV = 0x04;
constexpr __u8 OUTNE_1 = 0x02;
constexpr __u8 OUTNE_0 = 0x01;

constexpr double BASE_MULTIPLE = 4096;
constexpr __u8 MIN_PRESCALE = 0x03;
constexpr __u8 MAX_PRESCALE = 0xFF;
constexpr __u8 NUM_PWM = 16;

PCA9685::PCA9685() : PCA9685(DEFAULT_ADDRESS) {}

double PCA9685::getResolution() { return 1.0 / BASE_MULTIPLE; }

void PCA9685::start() {
    write_bit(MODE1, SLEEP, false);
    do {
        usleep(500);
    } while (isSleeping());
}

void PCA9685::sleep() {
    write_bit(MODE1, SLEEP, true);
}

bool PCA9685::isSleeping() {
    return read_bit(MODE1, SLEEP);
}

void PCA9685::restart() {
    __u8 m = read_byte_data(MODE1);
    if (m & SLEEP == 0) return;
    if (m & RESTART == 0) { start(); return; }
    m = m & ~SLEEP;     // Clear SLEEP bit
    write_byte_data(MODE1, m);
    usleep(500);        // Stabilize oscillator
    m = m | RESTART;    // Set RESTART bit
    write_byte_data(MODE1, m);
    do {
        usleep(100);
    } while (!read_bit(MODE1, RESTART)); // Wait for RESTART to clear
}

void PCA9685::setExternalClock(double freq) {
    if (read_bit(MODE1, EXTCLK)) return;
    write_bit(MODE1, SLEEP, true);
    write_bit(MODE1, SLEEP | EXTCLK, true);
    clk_freq = freq;
}

void PCA9685::setAutoIncrement(bool value) {
    write_bit(MODE1, AI, value);
    auto_inc = read_bit(MODE1, AI);
}

// Sub-address 1
void PCA9685::activateSubAddr1() {
    activateSubAddr1(DEFAULT_SUBADDR1);
}

void PCA9685::activateSubAddr1(__u8 subaddr) {
    write_byte_data(SUBADR1, subaddr << 1);
    write_bit(MODE1, SUB1, true);
}

void PCA9685::deactivateSubAddr1() {
    write_bit(MODE1, SUB1, false);
}

// Sub-address 2
void PCA9685::activateSubAddr2() {
    activateSubAddr2(DEFAULT_SUBADDR2);
}

void PCA9685::activateSubAddr2(__u8 subaddr) {
    write_byte_data(SUBADR2, subaddr << 1);
    write_bit(MODE1, SUB2, true);
}

void PCA9685::deactivateSubAddr2() {
    write_bit(MODE1, SUB2, false);
}

// Sub-address 3
void PCA9685::activateSubAddr3() {
    activateSubAddr3(DEFAULT_SUBADDR3);
}

void PCA9685::activateSubAddr3(__u8 subaddr) {
    write_byte_data(SUBADR3, subaddr << 1);
    write_bit(MODE1, SUB3, true);
}

void PCA9685::deactivateSubAddr3() {
    write_bit(MODE1, SUB3, false);
}

// All-call address
void PCA9685::activateAllCallAddr() {
    activateAllCallAddr(DEFAULT_ALLCALLADDR);
}

void PCA9685::activateAllCallAddr(__u8 subaddr) {
    write_byte_data(ALLCALLADR, subaddr << 1);
    write_bit(MODE1, ALLCALL, true);
}

void PCA9685::deactivateAllCallAddr() {
    write_bit(MODE1, ALLCALL, false);
}

// Mode 2 control
void PCA9685::setInvertedLogic(bool value) {
    write_bit(MODE2, INVRT, value);
}

void PCA9685::setOutputChangeOnStop() {
    write_bit(MODE2, OCH, false);
}

void PCA9685::setOutputChangeOnAck() {
    write_bit(MODE2, OCH, true);
}

void PCA9685::setOpenDrainOutputs() {
    write_bit(MODE2, OUTDRV, false);
}

void PCA9685::setTotemPoleOutputs() {
    write_bit(MODE2, OUTDRV, true);
}

void PCA9685::setOutputEnableEffect(PCA9685::OE_EFFECT effect) {
    __u8 m = read_byte_data(MODE2) & ~(OUTNE_1 | OUTNE_0);
    switch (effect) {
        case PCA9685::OE_EFFECT::LED_OFF: break;
        case PCA9685::OE_EFFECT::OUTDRV_CONTROL: m = m | OUTNE_0; break;
        case PCA9685::OE_EFFECT::LED_HIGH_IMPEDANCE: m = m | OUTNE_1; break;
    }
    write_byte_data(MODE2, m);
}

void PCA9685::setPWMFreq(double freq) {
    bool active = !isSleeping();
    if (active) this->sleep();

    double prescale = round(clk_freq / (BASE_MULTIPLE * freq)) - 1;
    if (prescale < MIN_PRESCALE) {
        std::cerr << "Cannot set PWM frequency to " << freq
            << " - requires a prescale value " << prescale
            << " which is lower than the minimum " << MIN_PRESCALE
            << std::endl;
    } else if (prescale > MAX_PRESCALE) {
        std::cerr << "Cannot set PWM frequency to " << freq
            << " - requires a prescale value " << prescale
            << " which is greater than the maximum " << MAX_PRESCALE
            << std::endl;
    } else {
        __u8 prescale_byte = static_cast<__u8>(prescale);
        write_byte_data(PRE_SCALE, prescale_byte);
        pwm_freq = clk_freq / (BASE_MULTIPLE * (prescale + 1));
    }

    if (active) this->restart();
}

void PCA9685::setPWM(__u8 index, uint16_t pwm_on, uint16_t pwm_off) {
    if (index > NUM_PWM) {
        std::cerr << "Invalid PWM index " << index << std::endl; return;
    }
    __u8 start_register{ 0 };
    if (index == NUM_PWM) {
        start_register = ALL_LED_ON_L;
    } else {
        start_register = LED0_ON_L + 4 * index;
    }
    vector<__u8> bytes(4, 0);
    bytes[0] = static_cast<__u8>(pwm_on & 0xFF);
    bytes[1] = static_cast<__u8>((pwm_on >> 8) & 0x1F);
    bytes[2] = static_cast<__u8>(pwm_off & 0xFF);
    bytes[3] = static_cast<__u8>((pwm_off >> 8) & 0x1F);
    if (auto_inc) {
        write_block_data(start_register, 4, bytes.data());
    } else {
        for (__u8 i = 0; i < 4; i++)
            write_byte_data(start_register + i, bytes[i]);
    }
}

void PCA9685::setPWMConstant(__u8 index, bool on) {
    const uint16_t full_flag = 0x1000;
    if (on) {
        setPWM(index, full_flag, 0);
    } else {
        setPWM(index, 0, full_flag);
    }
}

void PCA9685::setPWM(__u8 index, double phase, double duty) {
    if (phase < 0 || phase >= 1) {
        std::cerr << "Invalid PWM phase " << phase << std::endl; return;
    }
    if (duty < 0 || duty > 1) {
        std::cerr << "Invalid PWM duty cycle " << duty << std::endl; return;
    }

    double pwm_on_d = round(phase * BASE_MULTIPLE);
    if (pwm_on_d >= BASE_MULTIPLE) pwm_on_d = BASE_MULTIPLE - 1;
    uint16_t pwm_on = static_cast<uint16_t>(pwm_on_d);

    double pwm_off_d = round((phase + duty) * BASE_MULTIPLE);
    if (pwm_off_d >= BASE_MULTIPLE) pwm_off_d -= BASE_MULTIPLE;
    if (pwm_off_d >= BASE_MULTIPLE) pwm_off_d = BASE_MULTIPLE - 1;
    uint16_t pwm_off = static_cast<uint16_t>(pwm_off_d);

    setPWM(index, pwm_on, pwm_off);
}

void PCA9685::setAllPWMConstant(bool on) {
    setPWMConstant(NUM_PWM, on);
}

void PCA9685::setAllPWM(uint16_t pwm_on, uint16_t pwm_off) {
    setPWM(NUM_PWM, pwm_on, pwm_off);
}

void PCA9685::setAllPWM(double phase, double duty) {
    setPWM(NUM_PWM, phase, duty);
}