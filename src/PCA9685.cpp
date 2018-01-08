#include <PCA9685.hpp>

#include <unistd.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <bitset>
#include <chrono>
#include <functional>

using std::vector;

constexpr unsigned char DEFAULT_ADDRESS = 0x40;
constexpr unsigned char DEFAULT_OE_PIN  = 4;

// Register addresses
constexpr unsigned char MODE1        = 0x00;
constexpr unsigned char MODE2        = 0x01;
constexpr unsigned char SUBADR1      = 0x02;
constexpr unsigned char SUBADR2      = 0x03;
constexpr unsigned char SUBADR3      = 0x04;
constexpr unsigned char ALLCALLADR   = 0x05;
constexpr unsigned char LED0_ON_L    = 0x06;
    // Registers LEDi_ON_L, LEDi_ON_H, LEDi_OFF_L, LEDi_OFF_H etc...
constexpr unsigned char ALL_LED_ON_L = 0xFA;
    // Registers ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H
constexpr unsigned char PRE_SCALE    = 0xFE;

// Default software addresses
constexpr unsigned char DEFAULT_SUBADDR1 = 0x71;
constexpr unsigned char DEFAULT_SUBADDR2 = 0x72;
constexpr unsigned char DEFAULT_SUBADDR3 = 0x74;
constexpr unsigned char DEFAULT_ALLCALLADDR = 0x70;

// Mode 1 Bits
constexpr unsigned char RESTART = 0x80;
constexpr unsigned char EXTCLK = 0x40;
constexpr unsigned char AI = 0x20;
constexpr unsigned char SLEEP = 0x10;
constexpr unsigned char SUB1 = 0x08;
constexpr unsigned char SUB2 = 0x04;
constexpr unsigned char SUB3 = 0x02;
constexpr unsigned char ALLCALL = 0x01;

// Mode 2 Bits
constexpr unsigned char INVRT = 0x10;
constexpr unsigned char OCH = 0x08;
constexpr unsigned char OUTDRV = 0x04;
constexpr unsigned char OUTNE_1 = 0x02;
constexpr unsigned char OUTNE_0 = 0x01;

constexpr double BASE_MULTIPLE = 4096;
constexpr unsigned char MIN_PRESCALE = 0x03;
constexpr unsigned char MAX_PRESCALE = 0xFF;
constexpr unsigned char NUM_PWM = 16;

constexpr uint16_t EMPTY_FLAG = 0x0000;
constexpr uint16_t FULL_FLAG  = 0x1000;

constexpr useconds_t SLEEP_US = 500;

using clk = std::chrono::high_resolution_clock;

PCA9685::PCA9685(unsigned char address, unsigned char oePin) :
    i2cDevice(address),
    oePin(oePin),
    pwm_on_reg(NUM_PWM, EMPTY_FLAG), 
    pwm_off_reg(NUM_PWM, FULL_FLAG),
    last_on(clk::now()) {}

PCA9685::PCA9685() : PCA9685(DEFAULT_ADDRESS, DEFAULT_OE_PIN) {}

namespace {
    void killThreadPtr(std::unique_ptr<std::thread>& threadPtr) {
        if (threadPtr && threadPtr->joinable()) threadPtr->join();
        threadPtr.reset(nullptr);
    }

    using dur_ms = std::chrono::duration<size_t, std::milli>;
    void autoOffLoop
        (const clk::time_point& last_on, 
         const size_t& auto_off_ms,
         PCA9685* chip) 
    {
        while (true) {
            const dur_ms tgt_dur(auto_off_ms);
            if (tgt_dur.count() == 0) break;
            const clk::time_point curr_time = clk::now();
            const clk::time_point last_time = last_on;
            if (curr_time <= last_time) continue;
            const dur_ms actual_dur
                = std::chrono::duration_cast<dur_ms>(curr_time - last_time);
            if (actual_dur >= tgt_dur) {
                chip->setOutputEnable(false);
            } else {
                std::this_thread::sleep_for(tgt_dur);
            }
        }
    }
}

PCA9685::~PCA9685() {
    killThreadPtr(autoOffThread);
}

double PCA9685::getResolution() { return 1.0 / BASE_MULTIPLE; }

void PCA9685::forceOutputEnable() {
    last_on = clk::now();
    if (getOutputEnable()) return;
    setOutputEnable(true);
    usleep(SLEEP_US);
}

void PCA9685::setAutoOff(size_t ms) {
    last_on = clk::now();
    auto_off_ms = ms;
    if (ms > 0) {
        if (!autoOffThread) 
            autoOffThread.reset(new std::thread(
                autoOffLoop, last_on, auto_off_ms, this));
    } else {
        killThreadPtr(autoOffThread);
    }
}

void PCA9685::start() {
    forceOutputEnable();
    write_bit(MODE1, SLEEP, false);
    do {
        usleep(SLEEP_US);
    } while (isSleeping());
}

void PCA9685::sleep() {
    write_bit(MODE1, SLEEP, true);
    setOutputEnable(false);
}

bool PCA9685::isSleeping() {
    return read_bit(MODE1, SLEEP);
}

void PCA9685::restart() {
    forceOutputEnable();
    unsigned char m = read_byte_data(MODE1);
    if (m & SLEEP == 0) return;
    if (m & RESTART == 0) { start(); return; }
    m = m & ~SLEEP;     // Clear SLEEP bit
    write_byte_data(MODE1, m);
    usleep(SLEEP_US);   // Stabilize oscillator
    m = m | RESTART;    // Set RESTART bit
    write_byte_data(MODE1, m);
    do {
        usleep(SLEEP_US);
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

void PCA9685::activateSubAddr1(unsigned char subaddr) {
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

void PCA9685::activateSubAddr2(unsigned char subaddr) {
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

void PCA9685::activateSubAddr3(unsigned char subaddr) {
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

void PCA9685::activateAllCallAddr(unsigned char subaddr) {
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
    unsigned char m = read_byte_data(MODE2) & ~(OUTNE_1 | OUTNE_0);
    switch (effect) {
        case PCA9685::OE_EFFECT::LED_OFF: break;
        case PCA9685::OE_EFFECT::OUTDRV_CONTROL: m = m | OUTNE_0; break;
        case PCA9685::OE_EFFECT::LED_HIGH_IMPEDANCE: m = m | OUTNE_1; break;
    }
    write_byte_data(MODE2, m);
}

void PCA9685::setPWMFreq(double freq) {
    forceOutputEnable();

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
        unsigned char prescale_byte = static_cast<unsigned char>(prescale);
        write_byte_data(PRE_SCALE, prescale_byte);
        pwm_freq = clk_freq / (BASE_MULTIPLE * (prescale + 1));
    }

    if (active) this->restart();
}

void PCA9685::setPWM(unsigned char index, uint16_t pwm_on, uint16_t pwm_off) {
    if (index > NUM_PWM) {
        std::cerr << "Invalid PWM index " << index << std::endl; return;
    }
    forceOutputEnable();
    unsigned char start_register{ 0 };
    if (index == NUM_PWM) {
        start_register = ALL_LED_ON_L;
    } else {
        start_register = LED0_ON_L + 4 * index;
    }
    vector<unsigned char> bytes(4, 0);
    bytes[0] = static_cast<unsigned char>(pwm_on & 0xFF);
    bytes[1] = static_cast<unsigned char>((pwm_on >> 8) & 0x1F);
    bytes[2] = static_cast<unsigned char>(pwm_off & 0xFF);
    bytes[3] = static_cast<unsigned char>((pwm_off >> 8) & 0x1F);
    if (auto_inc) {
        write_block_data(start_register, 4, bytes.data());
    } else {
        for (unsigned char i = 0; i < 4; i++)
            write_byte_data(start_register + i, bytes[i]);
    }
    if (index == NUM_PWM) {
        std::fill(pwm_on_reg.begin(), pwm_on_reg.end(), pwm_on);
        std::fill(pwm_off_reg.begin(), pwm_off_reg.end(), pwm_off);
    } else {
        pwm_on_reg[index] = pwm_on;
        pwm_off_reg[index] = pwm_off;
    }
}

void PCA9685::setPWMConstant(unsigned char index, bool on) {
    if (on) {
        setPWM(index, FULL_FLAG, EMPTY_FLAG);
    } else {
        setPWM(index, EMPTY_FLAG, FULL_FLAG);
    }
}

void PCA9685::setPWM(unsigned char index, double phase, double duty) {
    if (phase < 0 || phase >= 1) {
        std::cerr << "Invalid PWM phase " << phase << std::endl; return;
    }
    if (duty < 0 || duty > 1) {
        std::cerr << "Invalid PWM duty cycle " << duty << std::endl; return;
    }
    if (duty == 0) {
        setPWMConstant(index, false); return;
    }
    if (duty == 1) {
        setPWMConstant(index, true); return;
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

void PCA9685::setPulseWidth(unsigned char index, double phase, double pulse_ms) {
    const double pwm_ms = 1000.0 / pwm_freq;
    if (pulse_ms < 0 || pulse_ms > pwm_ms) {
        std::cerr << "Cannot set invalid pulse width " << pulse_ms
                  << " ms" << std::endl; return;
    }
    const double duty_cycle = pulse_ms / pwm_ms;
    setPWM(index, phase, duty_cycle);
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

void PCA9685::setAllPulseWidth(double phase, double pulse_ms) {
    setPulseWidth(NUM_PWM, phase, pulse_ms);
}

double PCA9685::getPhase(unsigned char index) const {
    if (index >= NUM_PWM) {
        std::cerr << "Cannot get phase for invalid index " 
                  << (int)index << std::endl; 
        return 0.0;
    } else {
        const uint16_t pwm_on = pwm_on_reg[index];
        if (pwm_on == FULL_FLAG) return 0.0;
        return static_cast<double>(pwm_on) / BASE_MULTIPLE;
    }
}

double PCA9685::getDutyCycle(unsigned char index) const {
    if (index >= NUM_PWM) {
        std::cerr << "Cannot get duty cycle for invalid index "
            << (int)index << std::endl; 
        return 0.0;
    } else {
        const uint16_t pwm_on = pwm_on_reg[index];
        if (pwm_on == FULL_FLAG) return 1.0;
        const uint16_t pwm_off = pwm_off_reg[index];
        if (pwm_off == FULL_FLAG) return 0.0;
        const double pwm_on_d = static_cast<double>(pwm_on);
        const double pwm_off_d = static_cast<double>(pwm_off);
        double diff = pwm_off_d - pwm_on_d;
        diff = (diff < 0) ? diff + BASE_MULTIPLE : diff;
        return diff / BASE_MULTIPLE;
    }
}

double PCA9685::getPulseWidth(unsigned char index) const {
    const double pwm_ms = 1000.0 / pwm_freq;
    const double duty_cycle = getPulseWidth(index);
    return duty_cycle * pwm_ms;
}