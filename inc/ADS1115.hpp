#pragma once

#include <i2c.hpp>

class ADS1115 : protected i2cDevice {
private:
    bool continuousMode{ false };
    double fsr{ 2.048 };
    double low_thresh, high_thresh;
    uint16_t readConfig();
    void updateFSR(double value);

public:
    ADS1115(unsigned char address);
    ADS1115();
    ADS1115(const ADS1115& rhs) = delete;
    ADS1115(ADS1115&& rhs) = delete;

    enum class MultiplexerConfig : uint16_t {
        A0_MINUS_A1 = 0,
        A0_MINUS_A3 = 1,
        A1_MINUS_A3 = 2,
        A2_MINUS_A3 = 3,
        A0 = 4,
        A1 = 5,
        A2 = 6,
        A3 = 7
    };
    void setMultiplexerConfig
        (MultiplexerConfig multiplexerConfig = MultiplexerConfig::A0_MINUS_A1);

    enum class FullScaleRange : uint16_t {
        V_6144 = 0,
        V_4096 = 1,
        V_2048 = 2,
        V_1024 = 3,
        V_0512 = 4,
        V_0256 = 5
    };
    void setFullScaleRange(FullScaleRange range = FullScaleRange::V_2048);

    void setContinuousMode(bool value);

    enum class DataRate : uint16_t {
        SPS_8 = 0,
        SPS_16 = 1,
        SPS_32 = 2,
        SPS_64 = 3,
        SPS_128 = 4,
        SPS_250 = 5,
        SPS_475 = 6,
        SPS_860 = 7
    };
    void setDataRate(DataRate rate = DataRate::SPS_128);

    enum class ComparatorMode : uint16_t {
        TRADITIONAL = 0,
        WINDOW = 1
    };
    void setComparatorMode(ComparatorMode mode = ComparatorMode::TRADITIONAL);
    void setComparatorPolarity(bool activeHigh = false);
    void setComparatorLatching(bool latching = false);

    enum class ComparatorQueue : uint16_t {
        ONE = 0,
        TWO = 1,
        FOUR = 2,
        DISABLE = 3
    };
    void setComparatorQueue(ComparatorQueue queue);
    void disableComparator() { setComparatorQueue(ComparatorQueue::DISABLE); }

    void setup
        (MultiplexerConfig multiplexerConfig, FullScaleRange range,
         DataRate rate, bool continuousMode);

    void setupComparator
        (ComparatorMode mode, ComparatorQueue queue,
         bool activeHigh = false, bool latching = false);

    double getVoltage();
    void setLowThreshold(double voltage);
    void setHighThreshold(double voltage);
};