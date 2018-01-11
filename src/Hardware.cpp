#include <Hardware.hpp>
#include <i2c.hpp>
#include <iostream>

#include <ServoController.hpp>
#include <ADS1115.hpp>
#include <MPU9250.hpp>
#include <wiringPi.h>
#include <pigpio.h>
#include <thread>
#include <chrono>

namespace {
    constexpr unsigned char PAN_SERVO = 0;
    constexpr double PAN_SERVO_PHASE = 0;
    constexpr unsigned char TILT_SERVO = 15;
    constexpr double TILT_SERVO_PHASE = 0.5;
    const std::chrono::duration<size_t, std::milli> ADC_SWITCH_DUR(250);
    constexpr ADS1115::MultiplexerConfig ADC_POT_INPUT_CFG
        = ADS1115::MultiplexerConfig::A0_MINUS_A1;
    constexpr ADS1115::MultiplexerConfig ADC_LIGHT_INPUT_CFG
        = ADS1115::MultiplexerConfig::A3;
    constexpr unsigned int LED_PIN = 14;
    constexpr unsigned int L_BUTTON_PIN = 18;
    constexpr unsigned int R_BUTTON_PIN = 15;

    void switchAdcInput
        (std::unique_ptr<ADS1115>& adc, bool& adcPotInput, bool newValue) 
    {
        if (!adc || adcPotInput == newValue) return;
        if (newValue) {
            adc->setMultiplexerConfig(ADC_POT_INPUT_CFG);
        } else {
            adc->setMultiplexerConfig(ADC_LIGHT_INPUT_CFG);
        }
        std::this_thread::sleep_for(ADC_SWITCH_DUR);
        adcPotInput = newValue;
    }

    void buttonPressL_(int event, int level, uint32_t tick, void* userdata) {
        Hardware* hw = reinterpret_cast<Hardware*>(userdata);
        if (level != RISING_EDGE) return;
        hw->buttonPressL();
    }
    void buttonPressR_(int event, int level, uint32_t tick, void* userdata) {
        Hardware* hw = reinterpret_cast<Hardware*>(userdata);
        if (level != RISING_EDGE) return;
        hw->buttonPressR();
    }

    constexpr double POT_V_RANGE = 1.65;
}

Hardware::Hardware() : servoController(nullptr) {
    i2c::softwareReset();
    wiringPiSetup();
    gpioInitialise();

    gpioSetMode(LED_PIN, PI_OUTPUT);
    gpioSetMode(L_BUTTON_PIN, PI_INPUT);
    gpioSetMode(R_BUTTON_PIN, PI_INPUT);
    gpioSetAlertFuncEx(L_BUTTON_PIN, &buttonPressL_, this);
    gpioSetAlertFuncEx(R_BUTTON_PIN, &buttonPressR_, this);

    servoController.reset(new ServoController());

    adc.reset(new ADS1115());
    adc->disableComparator();
    adc->setup(
        ADC_LIGHT_INPUT_CFG,
        ADS1115::FullScaleRange::V_2048,
        ADS1115::DataRate::SPS_8,
        true);
    
    mpu.reset(new MPU9250());
    mpu->setSampleRate(10);
    mpu->setAccelDLPFConfig(MPU9250::AccelDLPFConfig::DLPF_CFG_5);
    mpu->disableGyroscope();
}

Hardware::~Hardware() = default;

void Hardware::setMirrorPan(double deg) {
    if (!servoController) return;
    if (deg < -90 || deg > 90) {
        std::cerr << "Invalid pan angle " << deg << std::endl; return;
    }
    const double pos = (deg + 90) / 180;
    servoController->setServoPosition(PAN_SERVO, PAN_SERVO_PHASE, pos);
}

void Hardware::setMirrorTilt(double deg) {
    if (!servoController) return;
    if (deg < 0 || deg > 90) {
        std::cerr << "Invalid tilt angle " << deg << std::endl; return;
    }
    const double pos = (deg + 90) / 180;
    servoController->setServoPosition(TILT_SERVO, TILT_SERVO_PHASE, pos);
}

double Hardware::getLightVoltage() {
    switchAdcInput(adc, adcPotInput, false);
    return adc ? adc->getVoltage() : 0.0;
}

double Hardware::getPotentiometerVoltage() {
    switchAdcInput(adc, adcPotInput, true);
    return adc ? adc->getVoltage() : 0.0;
}

namespace {
    const value3d zero_vector = { 0,0,0 };
}

value3d Hardware::getAcceleration() {
    if (!mpu) return zero_vector;
    return mpu->getAcceleration();
}

value3d Hardware::getMagnetometer() {
    if (!mpu) return zero_vector;
    return mpu->getMagnetometer();
}

double Hardware::getTemperature() {
    if (!mpu) return 0;
    return mpu->getTemperature();
}

void Hardware::buttonPressL() {
    bool EXPECTED = false;
    buttonL.compare_exchange_strong(EXPECTED, true);
}

void Hardware::buttonPressR() {
    bool EXPECTED = false;
    buttonR.compare_exchange_strong(EXPECTED, true);
}

void Hardware::calibratePan() {
    switchAdcInput(adc, adcPotInput, true);
    setMirrorTilt(90);
    bool BUTTON_EXPECTED = true;
    while (true) {
        double potV = getPotentiometerVoltage();
        double pos = potV / POT_V_RANGE;
        if (pos < -1) pos = -1;
        if (pos > 1) pos = 1;
        pos = (1 + pos) / 2;
        servoController->setServoPosition(PAN_SERVO, PAN_SERVO_PHASE, pos);
        if (buttonL.compare_exchange_strong(BUTTON_EXPECTED, false)) break;
    }
}