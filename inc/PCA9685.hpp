#pragma once

#include <i2c.hpp>
#include <GPIOPin.hpp>
#include <cstdint>
#include <vector>
#include <chrono>
#include <memory>
#include <thread>

class PCA9685 : protected i2cDevice {
private:
    GPIOPin oePin;
    bool auto_inc{ false };
    double clk_freq{ 25e6 };
    std::vector<uint16_t> pwm_on_reg;
    std::vector<uint16_t> pwm_off_reg;
    double pwm_freq{ 200 };
    std::chrono::high_resolution_clock::time_point last_on;
    size_t auto_off_ms{ 0 };
    std::unique_ptr<std::thread> autoOffThread{ nullptr };

public:
    PCA9685(unsigned char address, unsigned char oePin);
    PCA9685();
    ~PCA9685();
    PCA9685(const PCA9685& rhs) = delete;
    PCA9685(PCA9685&& rhs) = delete;

    bool getOutputEnable() { return !oePin.getValue(); }
    void setOutputEnable(bool enable) { return oePin.setValue(!enable); }
    void forceOutputEnable();

    void setAutoOff(size_t ms);
    void disableAutoOff() { setAutoOff(0); }

    void start();
    void sleep();
    bool isSleeping();
    void restart();

    double getClockFrequency() { return clk_freq; }
    double getPWMFrequency() { return pwm_freq; }
    double getResolution();

    void setExternalClock(double freq);
    void setAutoIncrement(bool value = true);

    void activateSubAddr1();
    void activateSubAddr1(unsigned char subaddr);
    void deactivateSubAddr1();

    void activateSubAddr2();
    void activateSubAddr2(unsigned char subaddr);
    void deactivateSubAddr2();

    void activateSubAddr3();
    void activateSubAddr3(unsigned char subaddr);
    void deactivateSubAddr3();

    void activateAllCallAddr();
    void activateAllCallAddr(unsigned char subaddr);
    void deactivateAllCallAddr();

    void setInvertedLogic(bool value = true);
    void setOutputChangeOnStop();
    void setOutputChangeOnAck();
    void setOpenDrainOutputs();
    void setTotemPoleOutputs();

    enum class OE_EFFECT {
        LED_OFF,
        OUTDRV_CONTROL,
        LED_HIGH_IMPEDANCE
    };
    void setOutputEnableEffect(OE_EFFECT effect = OE_EFFECT::LED_OFF);

    void setPWMFreq(double freq);

    void setPWMConstant(unsigned char index, bool on);
    void setAllPWMConstant(bool on);

    void setPWM(unsigned char index, uint16_t pwm_on, uint16_t pwm_off);
    void setAllPWM(uint16_t pwm_on, uint16_t pwm_off);

    void setPWM(unsigned char index, double phase, double duty);
    void setAllPWM(double phase, double duty);

    void setPulseWidth(unsigned char index, double phase, double pulse_ms);
    void setAllPulseWidth(double phase, double pulse_ms);

    double getPWMFreq() const { return pwm_freq; }
    double getPhase(unsigned char index) const;
    double getDutyCycle(unsigned char index) const;
    double getPulseWidth(unsigned char index) const;
};