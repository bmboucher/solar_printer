#include <Hardware.hpp>
#include <i2c.hpp>
#include <iostream>

#include <ServoController.hpp>
#include <ADS1115.hpp>
#include <MPU9250.hpp>
#include <wiringPi.h>
#include <pigpio.h>

constexpr unsigned char PAN_SERVO = 0;
constexpr double PAN_SERVO_PHASE = 0;
constexpr unsigned char TILT_SERVO = 15;
constexpr double TILT_SERVO_PHASE = 0.5;

Hardware::Hardware() : servoController(nullptr) {
    i2c::softwareReset();
    wiringPiSetup();
    gpioInitialise();

    servoController.reset(new ServoController());

    adc.reset(new ADS1115());
    adc->disableComparator();
    adc->setup(
        ADS1115::MultiplexerConfig::A0_MINUS_A1,
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

double Hardware::getVoltage() {
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