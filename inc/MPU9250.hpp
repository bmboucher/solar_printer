#pragma once

#include <array>
#include <cstdint>

using value3d = std::array<double, 3>;

class MPU9250 {
private:
    double gyro_fs;
    double accel_fs;
    value3d mag_sa;

public:
    MPU9250();

    void setSampleRate(double rate);

    enum class GyroFullScale : uint8_t {
        DPS_250 = 0, DPS_500 = 1, DPS_1000 = 2, DPS_2000 = 3
    };
    void setGyroFullScale(GyroFullScale scale);

    enum class GyroDLPFConfig : uint8_t {
                            //       GYROSCOPE            TEMPERATURE
                            // BW(Hz)  Delay  Fs(kHz)    BW(Hz)  Delay
        FCHOICE_0  = 0x08,  //  8800   0.064    32        4000    0.04
        FCHOICE_1  = 0x10,  //  3600    0.11    32        4000    0.04
        DLPF_CFG_0 = 0x00,  //   250    0.97     8        4000    0.04
        DLPF_CFG_1 = 0x01,  //   184    2.9      1         188    1.9
        DLPF_CFG_2 = 0x02,  //    92    3.9      1          98    2.8
        DLPF_CFG_3 = 0x03,  //    41    5.9      1          42    4.8
        DLPF_CFG_4 = 0x04,  //    20    9.9      1          20    8.3
        DLPF_CFG_5 = 0x05,  //    10   17.85     1          10   13.4
        DLPF_CFG_6 = 0x06,  //     5   33.48     1           5   18.6
        DLPF_CFG_7 = 0x07   //  3600    0.17     8        4000    0.04
    };
    void setGyroDLPFConfig(GyroDLPFConfig config);

    enum class AccelFullScale : uint8_t {
        G_2 = 0, G_4 = 1, G_8 = 2, G_16 = 3
    };
    void setAccelFullScale(AccelFullScale scale);

    enum class AccelDLPFConfig : uint8_t {
                            // BW(Hz)  Delay  Rate(kHz)
        FCHOICE    = 0x08,  //  1130    0.75      4
        DLPF_CFG_0 = 0x00,  //   460    1.94      1
        DLPF_CFG_1 = 0x01,  //   184    5.80      1
        DLPF_CFG_2 = 0x02,  //    92    7.80      1
        DLPF_CFG_3 = 0x03,  //    41   11.80      1
        DLPF_CFG_4 = 0x04,  //    20   19.80      1
        DLPF_CFG_5 = 0x05,  //    10   35.70      1
        DLPF_CFG_6 = 0x06,  //     5   66.96      1
        DLPF_CFG_7 = 0x07   //   460    1.94      1
    };
    void setAccelDLPFConfig(AccelDLPFConfig config);

    value3d getAcceleration();

    double getTemperature();

    value3d getGyroscope();

    value3d getMagnetometer();

    enum class MagnetometerDataRate : uint8_t {
        HZ_8   = 0x02,
        HZ_100 = 0x06
    };
    void setMagnetometerDataRate
        (MagnetometerDataRate rate = MagnetometerDataRate::HZ_8);
};