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
#include <cmath>

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
    gpioSetPullUpDown(L_BUTTON_PIN, PI_PUD_DOWN);
    gpioSetPullUpDown(R_BUTTON_PIN, PI_PUD_DOWN);
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
    double pos = panZero;
    if (deg < 0) {
        pos = panMinus90 + (panZero - panMinus90) * (deg + 90) / 90;
    } else {
        pos = panZero + (panPlus90 - panZero) * (deg / 90);
    }
    servoController->setServoPosition(PAN_SERVO, PAN_SERVO_PHASE, pos);
}

void Hardware::setMirrorTilt(double deg) {
    if (!servoController) return;
    if (deg < 0 || deg > 90) {
        std::cerr << "Invalid tilt angle " << deg << std::endl; return;
    }
    const double pos = tiltZero + (tilt90 - tiltZero) * (deg / 90);
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

namespace {
    double calibratePosition_(
            std::unique_ptr<ADS1115>& adc,
            std::unique_ptr<ServoController>& servoController,
            const uint8_t servo, const double phase,
            std::atomic_bool& buttonL) {
        buttonL.store(false);
        while (true) {
            double potV = adc->getVoltage();
            double pos = potV / POT_V_RANGE;
            if (pos < -1) pos = -1;
            if (pos > 1) pos = 1;
            pos = (1 + pos) / 2;
            servoController->setServoPosition(servo, phase, pos);
            bool BUTTON_EXPECTED{ true };
            if (buttonL.compare_exchange_weak(BUTTON_EXPECTED, false)) {
                double position = servoController->getServoPosition(servo);
                std::cout << "Position = " << position << std::endl;
                return position;
            }
        }
    }
}

void Hardware::calibrateTilt() {
    std::cout << "TILT CALIBRATION" << std::endl << std::endl;
    switchAdcInput(adc, adcPotInput, true);
    setMirrorTilt(90);
    setMirrorPan(0);
    const std::chrono::duration<size_t, std::milli> SETUP_TIME(1000);
    std::this_thread::sleep_for(SETUP_TIME);
    gpioWrite(LED_PIN, 1);

    auto calibratePosition = [this]() -> double
    {
        return calibratePosition_(
            adc, servoController, TILT_SERVO, TILT_SERVO_PHASE, buttonL);
    };

    std::cout << "Use pot to set position to 0" << std::endl;
    tiltZero = calibratePosition();

    std::cout << "Use pot to set position to 90" << std::endl;
    tilt90 = calibratePosition();

    setMirrorTilt(90);
    gpioWrite(LED_PIN, 0);
    std::cout << "TILT CALIBRATION COMPLETE" << std::endl << std::endl;
}

void Hardware::calibratePan() {
    std::cout << "PAN CALIBRATION" << std::endl << std::endl;

    switchAdcInput(adc, adcPotInput, true);
    setMirrorTilt(90);
    setMirrorPan(0);
    const std::chrono::duration<size_t, std::milli> SETUP_TIME(1000);
    std::this_thread::sleep_for(SETUP_TIME);
    gpioWrite(LED_PIN, 1);

    auto calibratePosition = [this]() -> double
    {
        return calibratePosition_(
            adc, servoController, PAN_SERVO, PAN_SERVO_PHASE, buttonL);
    };

    std::cout << "Use pot to set position to -90" << std::endl;
    panMinus90 = calibratePosition();
    
    std::cout << "Use pot to set position to 0" << std::endl;
    panZero = calibratePosition();

    std::cout << "Use pot to set position to +90" << std::endl;
    panPlus90 = calibratePosition();

    setMirrorPan(0);
    gpioWrite(LED_PIN, 0);

    std::cout << "PAN CALIBRATION COMPLETE" << std::endl << std::endl;
}

constexpr double FIGURE_EIGHT_MIN_TILT = 30;
constexpr double FIGURE_EIGHT_MAX_TILT = 80;
constexpr double FIGURE_EIGHT_PAN = 75;
void Hardware::figureEight() {
    // x^4 = a^2 * (x^2 - b * y^2)
    const double y_offset 
        = (FIGURE_EIGHT_MIN_TILT + FIGURE_EIGHT_MAX_TILT) / 2;
    const double y_semimajor
        = FIGURE_EIGHT_MAX_TILT - y_offset;
    const double a = FIGURE_EIGHT_PAN;
    const double b = pow(a / (2 * y_semimajor), 2);

    const std::chrono::duration<size_t, std::milli> t_step(10);
    double x_step = 1;
    double y_sign = 1;
    double x = -a;
    buttonL.store(false);
    while (true) {
        y = y_sign * sqr((pow(x, 2) - pow(x, 4) / pow(a, 2)) / b);
        setMirrorPan(x);
        setMirrorTilt(y + y_offset);
        std::this_thread::sleep_for(t_step);

        x += x_step;
        if (x + x_step > a || x + x_step < -a) 
            { x_step *= -1; y_sign *= -1; }
        if (x * (x + x_step) <= 0) { y_sign *= -1; }

        bool BUTTON_EXPECTED{ true };
        if (buttonL.compare_exchange_weak(BUTTON_EXPECTED, false)) return;
    }
}