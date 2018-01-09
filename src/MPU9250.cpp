#include <MPU9250.hpp>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <bitset>

using std::vector;
using std::array;
using byteset = std::bitset<8>;

namespace {
    constexpr uint8_t I2C_RD_BIT = 0x80;

    constexpr int SPI_CHANNEL = 0;
    constexpr int SPI_SPEED = 1000000;
    int spi() {
        static int spi = wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED);
        return spi;
    }

    uint8_t writeRegister(uint8_t reg, uint8_t value) {
        if (spi() < 0) {
            std::cerr << "Unable to open SPI" << std::endl; return 0;
        }
        vector<uint8_t> buffer(2, 0);
        buffer[0] = reg & ~I2C_RD_BIT;
        buffer[1] = value;
        if (wiringPiSPIDataRW(SPI_CHANNEL, buffer.data(), 2) < 0) {
            std::cerr << "Error sending value " << byteset(value)
                      << " to register " << byteset(reg) << std::endl;
            return 0;
        } else {
            return buffer[1];
        }
    }

    void writeBlockData(uint8_t reg, size_t n_bytes, const uint8_t* values) {
        if (spi() < 0) {
            std::cerr << "Unable to open SPI" << std::endl; return;
        }
        vector<uint8_t> buffer(n_bytes + 1, 0);
        buffer[0] = reg & ~I2C_RD_BIT;
        std::copy(values, values + n_bytes, &buffer[1]);
        if (wiringPiSPIDataRW(SPI_CHANNEL, buffer.data(), n_bytes + 1) < 0) {
            std::cerr << "Error sending " << n_bytes << " bytes"
                << " to register " << byteset(reg) << std::endl;
        }
    }

    uint8_t readRegister(uint8_t reg) {
        if (spi() < 0) {
            std::cerr << "Unable to open SPI" << std::endl; return 0;
        }
        vector<uint8_t> buffer(2, 0);
        buffer[0] = reg | I2C_RD_BIT;
        if (wiringPiSPIDataRW(SPI_CHANNEL, buffer.data(), 2) < 0) {
            std::cerr << "Error reading register " 
                      << byteset(reg) << std::endl;
            return 0;
        } else {
            return buffer[1];
        }
    }

    void readBlockData(uint8_t reg, size_t n_bytes, uint8_t* values) {
        if (spi() < 0) {
            std::cerr << "Unable to open SPI" << std::endl; return;
        }
        vector<uint8_t> buffer(n_bytes + 1, 0);
        buffer[0] = reg | I2C_RD_BIT;
        if (wiringPiSPIDataRW(SPI_CHANNEL, buffer.data(), n_bytes + 1) < 0) {
            std::cerr << "Error reading register " 
                      << byteset(reg) << std::endl;
        } else {
            std::copy(&buffer[1], &buffer[n_bytes], values);
        }
    }

    void setRegisterBit(uint8_t reg, uint8_t bitmask, bool value) {
        const uint8_t curr_value = readRegister(reg);
        uint8_t new_value = curr_value;
        if (value) {
            new_value |= bitmask;
        } else {
            new_value &= ~bitmask;
        }
        writeRegister(reg, new_value);
    }

    void setRegisterBits
        (uint8_t reg, uint8_t value, uint8_t bitmask, uint8_t shift = 0) 
    {
        uint8_t curr_value = readRegister(reg);
        curr_value &= ~bitmask;
        curr_value |= ((value << shift) & bitmask);
        writeRegister(reg, curr_value);
    }

    constexpr uint8_t SELF_TEST_X_GYRO  = 0x00; // ... SELF_TEST_Z_GYRO
    constexpr uint8_t SELF_TEST_X_ACCEL = 0x0D; // ... SELF_TEST_Z_ACCEL
    constexpr uint8_t XG_OFFSET_H       = 0x13; // ... ZG_OFFSET_L
    constexpr uint8_t SMPLRT_DIV        = 0x19;
    constexpr uint8_t CONFIG            = 0x1A;
    constexpr uint8_t GYRO_CONFIG       = 0x1B;
    constexpr uint8_t ACCEL_CONFIG      = 0x1C;
    constexpr uint8_t ACCEL_CONFIG_2    = 0x1D;
    constexpr uint8_t LP_ACCEL_ODR      = 0x1E;
    constexpr uint8_t WOM_THR           = 0x1F;
    constexpr uint8_t FIFO_EN           = 0x23;
    constexpr uint8_t I2C_MST_CTRL      = 0x24;
    constexpr uint8_t I2C_SLV0_ADDR     = 0x25; // ... I2C_SLV3_ADDR (every 3)
    constexpr uint8_t I2C_SLV0_REG      = 0x26; // ... I2C_SLV3_REG (every 3)
    constexpr uint8_t I2C_SLV0_CTRL     = 0x27; // ... I2C_SLV3_CTRL (every 3)
    constexpr uint8_t I2C_SLV4_ADDR     = 0x31;
    constexpr uint8_t I2C_SLV4_REG      = 0x32;
    constexpr uint8_t I2C_SLV4_DO       = 0x33;
    constexpr uint8_t I2C_SLV4_CTRL     = 0x34;
    constexpr uint8_t I2C_SLV4_DI       = 0x35;
    constexpr uint8_t I2C_MST_STATUS    = 0x36;
    constexpr uint8_t INT_PIN_CFG       = 0x37;
    constexpr uint8_t INT_ENABLE        = 0x38;
    constexpr uint8_t INT_STATUS        = 0x3A;
    constexpr uint8_t ACCEL_XOUT_H      = 0x3B; // ... ACCEL_ZOUT_L
    constexpr uint8_t TEMP_OUT_H        = 0x41; // ... TEMP_OUT_L
    constexpr uint8_t GYRO_XOUT_H       = 0x43; // ... GYRO_ZOUT_L
    constexpr uint8_t EXT_SENS_DATA_00  = 0x49; // ... EXT_SENS_DATA_23
    constexpr uint8_t I2C_SLV0_D0       = 0x63; // ... I2C_SLV3_D0
    constexpr uint8_t I2C_MST_DELAY_CTRL = 0x67;
    constexpr uint8_t SIGNAL_PATH_RESET = 0x68;
    constexpr uint8_t MOT_DETECT_CTRL   = 0x69;
    constexpr uint8_t USER_CTRL         = 0x6A;
    constexpr uint8_t PWR_MGMT_1        = 0x6B;
    constexpr uint8_t PWR_MGMT_2        = 0x6C;
    constexpr uint8_t FIFO_COUNTH       = 0x72; // ... FIFO_COUNTL
    constexpr uint8_t FIFO_R_W          = 0x74;
    constexpr uint8_t WHO_AM_I          = 0x75;
    constexpr uint8_t XA_OFFSET_H       = 0x77; // ... XA_OFFSET_L
    constexpr uint8_t YA_OFFSET_H       = 0x7A; // ... YA_OFFSET_L
    constexpr uint8_t ZA_OFFSET_H       = 0x7D; // ... ZA_OFFSET_L

    // AK8963 Magnetometer - communicates via I2C
    constexpr uint8_t AK8963_I2C_ADDR   = 0x0C;
    constexpr uint8_t AK8963_WIA        = 0x00;
    constexpr uint8_t AK8963_INFO       = 0x01;
    constexpr uint8_t AK8963_ST1        = 0x02;
    constexpr uint8_t AK8963_HXL        = 0x03; // ... AK8963_HZH
    constexpr uint8_t AK8963_ST2        = 0x09;
    constexpr uint8_t AK8963_CNTL       = 0x0A;
    constexpr uint8_t AK8963_ASTC       = 0x0C;
    constexpr uint8_t AK8963_I2CDIS     = 0x0F;
    constexpr uint8_t AK8963_ASAX       = 0x10; // ... AK8963_ASAZ

    // I2C_SLVi_CTRL bits
    constexpr uint8_t I2C_SLV_EN          = 0x80;
    constexpr uint8_t I2C_SLV_BYTE_SW     = 0x40;
    constexpr uint8_t I2C_SLV_REG_DIS     = 0x20;
    constexpr uint8_t I2C_SLV_GRP         = 0x10;
    constexpr uint8_t I2C_SLV_LEN_BITMASK = 0x0f;
    
    constexpr uint8_t MAG_READ_BYTES = 6; // 2 per axis, 3 axes
    void setupMagnetometerPolling() {
        // Setup SLV0 to continuously poll registers HXL..HZH
        writeRegister(I2C_SLV0_ADDR, I2C_RD_BIT | AK8963_I2C_ADDR);
        writeRegister(I2C_SLV0_REG, AK8963_HXL);
        // Swap HI/LOW bytes (I2C_SLV_BYTE_SW)
        // Group ends on even register (I2C_SLV_GRP)
        writeRegister(I2C_SLV0_CTRL, 
            I2C_SLV_EN | I2C_SLV_BYTE_SW | I2C_SLV_GRP | MAG_READ_BYTES);
    }

    constexpr useconds_t SLEEP_US = 10000;
    void writeMagnetometerRegister(uint8_t reg, uint8_t value) {
        vector<uint8_t> msg(4, 0);
        msg[0] = ~I2C_RD_BIT & AK8963_I2C_ADDR; // I2C_SLV4_ADDR
        msg[1] = reg;                           // I2C_SLV4_REG
        msg[2] = value;                         // I2C_SLV4_DO
        msg[3] = I2C_SLV_EN;                    // I2C_SLV4_CTRL
        writeBlockData(I2C_SLV4_ADDR, 4, msg.data());
        while (readRegister(I2C_SLV4_CTRL) & I2C_SLV_EN != 0) {
            usleep(SLEEP_US);
        }
    }
    uint8_t readMagnetometerRegister(uint8_t reg) {
        vector<uint8_t> msg(4, 0);
        msg[0] = I2C_RD_BIT | AK8963_I2C_ADDR;  // I2C_SLV4_ADDR
        msg[1] = reg;                           // I2C_SLV4_REG
        msg[2] = 0x00;                          // I2C_SLV4_DO
        msg[3] = I2C_SLV_EN;                    // I2C_SLV4_CTRL
        writeBlockData(I2C_SLV4_ADDR, 4, msg.data());
        while (readRegister(I2C_SLV4_CTRL) & I2C_SLV_EN != 0) {
            usleep(SLEEP_US);
        }
        return readRegister(I2C_SLV4_DI);
    }

    constexpr uint8_t GYRO_FS_BITMASK = 0x18;
    constexpr uint8_t GYRO_FS_SHIFT   = 3;

    constexpr uint8_t DLPF_CFG_BITMASK = 0x07;
    constexpr uint8_t GYRO_FCHOICE_BITMASK = 0x03;

    constexpr uint8_t ACCEL_FS_BITMASK = 0x18;
    constexpr uint8_t ACCEL_FS_SHIFT   = 3;

    // USER_CTRL bits
    constexpr uint8_t FIFO_EN      = 0x40;
    constexpr uint8_t I2C_MST_EN   = 0x20;
    constexpr uint8_t I2C_IF_DIS   = 0x10;
    constexpr uint8_t FIFO_RST     = 0x04;
    constexpr uint8_t I2C_MST_RST  = 0x02;
    constexpr uint8_t SIG_COND_RST = 0x01;

    constexpr double BASE_SAMPLE_RATE = 1000; // 1 kHz
    constexpr double BASE_GYRO_FS = 250;
    constexpr double BASE_ACCEL_FS = 2;

    constexpr double BASE_MULTIPLE = (1 << 15);
    double convertBytes(const uint8_t high_byte, const uint8_t low_byte) {
        const uint16_t word 
            = (static_cast<uint16_t>(high_byte) << 8)
                | static_cast<uint16_t>(low_byte);
        const int16_t signedWord = *reinterpret_cast<int16_t*>(&word);
        return static_cast<double>(signedWord) / BASE_MULTIPLE;
    }

    value3d convertBytes(const array<uint8_t, 6>& bytes, double scale) {
        value3d values;
        for (size_t i = 0; i < 3; i++) {
            const double fraction 
                = convertBytes(bytes[2 * i], bytes[2 * i + 1]);
            values[i] = fraction * scale;
        }
        return std::move(values);
    }
}

MPU9250::MPU9250() 
        : gyro_fs(BASE_GYRO_FS), 
          accel_fs(BASE_ACCEL_FS)
{
    if (spi() < 0) {
        std::cerr << "Unable to open SPI" << std::endl; return;
    }
    writeRegister(USER_CTRL, I2C_IF_DIS | SIG_COND_RST);
    setupMagnetometerPolling();
}

void MPU9250::setSampleRate(double rate) {
    if (rate >= BASE_SAMPLE_RATE) {
        std::cerr << "Cannot set sample rate to "
                  << rate << " which is higher than "
                  << "the base rate of " << BASE_SAMPLE_RATE << std::endl;
        return;
    }
    double divider = round(BASE_SAMPLE_RATE / rate) - 1;
    if (divider > 0xff) {
        std::cerr << "Cannot set sample rate to "
                  << rate << " - rate is too low" << std::endl;
        return;
    }
    uint8_t div_byte = static_cast<uint8_t>(divider);
    writeRegister(SMPLRT_DIV, div_byte);
}

void MPU9250::setGyroFullScale(GyroFullScale scale) {
    const uint8_t scale_value = static_cast<uint8_t>(scale);
    setRegisterBits(GYRO_CONFIG, scale_value,
                    GYRO_FS_BITMASK, GYRO_FS_SHIFT);
    gyro_fs = (BASE_GYRO_FS) * (1 << scale_value);
}

void MPU9250::setGyroDLPFConfig(GyroDLPFConfig config) {
    const uint8_t dlpf_value = static_cast<uint8_t>(config);
    setRegisterBits(CONFIG, dlpf_value, DLPF_CFG_BITMASK);
    const uint8_t fchoice_value = dlpf_value >> 3;
    setRegisterBits(GYRO_CONFIG, fchoice_value, GYRO_FCHOICE_BITMASK);
}

void MPU9250::setAccelFullScale(AccelFullScale scale) {
    const uint8_t scale_value = static_cast<uint8_t>(scale);
    setRegisterBits(ACCEL_CONFIG, scale_value,
                    ACCEL_FS_BITMASK, ACCEL_FS_SHIFT);
    accel_fs = (BASE_ACCEL_FS) * (1 << scale_value);
}

void MPU9250::setAccelDLPFConfig(AccelDLPFConfig config) {
    const uint8_t config_value = static_cast<uint8_t>(config);
    writeRegister(ACCEL_CONFIG_2, config_value);
}

value3d MPU9250::getAcceleration() {
    array<uint8_t, 6> buffer;
    readBlockData(ACCEL_XOUT_H, 6, buffer.data());
    return convertBytes(buffer, accel_fs);
}

constexpr double BASE_TEMPERATURE = 21;
constexpr double ROOM_TEMP_OFFSET = 0;
constexpr double TEMP_SENSITIVITY = 333.87 / BASE_MULTIPLE;
double MPU9250::getTemperature() {
    array<uint8_t, 2> buffer;
    readBlockData(TEMP_OUT_H, 2, buffer.data());
    const double fraction = convertBytes(buffer[0], buffer[1]);
    return ((fraction - ROOM_TEMP_OFFSET) / TEMP_SENSITIVITY)
                    + BASE_TEMPERATURE;
}

value3d MPU9250::getGyroscope() {
    array<uint8_t, 6> buffer;
    readBlockData(GYRO_XOUT_H, 6, buffer.data());
    return convertBytes(buffer, gyro_fs);
}

constexpr double MAG_FLUX_BASE = 4092;
value3d MPU9250::getMagnetometer() {
    array<uint8_t, 6> buffer;
    readBlockData(EXT_SENS_DATA_00, 6, buffer.data());
    return convertBytes(buffer, MAG_FLUX_BASE);
}