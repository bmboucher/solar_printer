#pragma once

#include <i2cDevice.hpp>
#include <cstdint>

class PCA9685 : protected i2cDevice {
private:
    bool auto_inc{ false };
    double clk_freq{ 25e6 };

protected:
    double pwm_freq{ 200 };

public:
    PCA9685(__u8 address) : i2cDevice(address) {}
    PCA9685();

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
    void activateSubAddr1(__u8 subaddr);
    void deactivateSubAddr1();

    void activateSubAddr2();
    void activateSubAddr2(__u8 subaddr);
    void deactivateSubAddr2();

    void activateSubAddr3();
    void activateSubAddr3(__u8 subaddr);
    void deactivateSubAddr3();

    void activateAllCallAddr();
    void activateAllCallAddr(__u8 subaddr);
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

    void setPWMConstant(__u8 index, bool on);
    void setAllPWMConstant(bool on);

    void setPWM(__u8 index, uint16_t pwm_on, uint16_t pwm_off);
    void setAllPWM(uint16_t pwm_on, uint16_t pwm_off);

    void setPWM(__u8 index, double phase, double duty);
    void setAllPWM(double phase, double duty);
};