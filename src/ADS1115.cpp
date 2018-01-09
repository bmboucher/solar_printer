#include <ADS1115.hpp>
#include <unistd.h>
#include <cmath>

constexpr unsigned char DEFAULT_ADDRESS = 0x48;

ADS1115::ADS1115(unsigned char address) : i2cDevice(address) {}
ADS1115::ADS1115() : ADS1115(DEFAULT_ADDRESS) {}

namespace {
    // Registers
    constexpr unsigned char CONVERSION = 0x00;
    constexpr unsigned char CONFIG     = 0x01;
    constexpr unsigned char LO_THRESH  = 0x02;
    constexpr unsigned char HI_THRESH  = 0x03;

    // Config register bits
    constexpr uint16_t OS                          = 0x8000;
    constexpr uint16_t MULTIPLEXER_CONFIG_BITMASK  = 0x8fff;
    constexpr unsigned char MULTIPLEXER_CONFIG_BIT = 12;
    constexpr uint16_t FSR_BITMASK                 = 0xf1ff;
    constexpr unsigned char FSR_BIT                = 9;
    constexpr uint16_t MODE_BIT                    = 0x0100;
    constexpr uint16_t DATA_RATE_BITMASK           = 0xff1f;
    constexpr unsigned char DATA_RATE_BIT          = 5;
    constexpr uint16_t COMP_MODE                   = 0x0010;
    constexpr uint16_t COMP_POL                    = 0x0008;
    constexpr uint16_t COMP_LAT                    = 0x0004;
    constexpr uint16_t COMP_QUE_BITMASK            = 0x0003;

    constexpr useconds_t SLEEP_US = 100;

    constexpr double BASE_MULTIPLE = (1 << 15);
    constexpr double ONE_MINUS_EPSILON = (BASE_MULTIPLE - 1) / BASE_MULTIPLE;
    constexpr uint16_t MIN_FRACTION = 0x8000;
    constexpr uint16_t MAX_FRACTION = 0x7fff;

    void applyMultiplexerConfig
        (uint16_t& config, uint16_t multiplexerConfig)
    {
        config &= MULTIPLEXER_CONFIG_BITMASK;
        config |= (multiplexerConfig << MULTIPLEXER_CONFIG_BIT);
    }

    void applyFullScaleRange
        (uint16_t& config, uint16_t range) 
    {
        config &= FSR_BITMASK;
        config |= (range << FSR_BIT);
    }

    void applyDataRate(uint16_t& config, uint16_t rate) {
        config &= DATA_RATE_BITMASK;
        config |= (rate << DATA_RATE_BIT);
    }

    void setWordBit(uint16_t& word, uint16_t bitmask, bool value) {
        if (value) {
            word |= bitmask;
        } else {
            word &= ~bitmask;
        }
    }

    void applyContinuousMode(uint16_t& config, bool value) {
        setWordBit(config, MODE_BIT, !value);
    }

    void applyComparatorMode(uint16_t& config, bool window) {
        setWordBit(config, COMP_MODE, window);
    }

    void applyComparatorPolarity(uint16_t& config, bool activeHigh) {
        setWordBit(config, COMP_POL, activeHigh);
    }

    void applyComparatorLatching(uint16_t& config, bool latching) {
        setWordBit(config, COMP_LAT, latching);
    }

    void applyComparatorQueue(uint16_t& config, uint16_t queue) {
        config &= COMP_QUE_BITMASK;
        config |= queue;
    }

    double getFSRValue(ADS1115::FullScaleRange range) {
        switch (range) {
            case ADS1115::FullScaleRange::V_6144: return 6.144;
            case ADS1115::FullScaleRange::V_4096: return 4.096;
            case ADS1115::FullScaleRange::V_2048: return 2.048;
            case ADS1115::FullScaleRange::V_1024: return 1.024;
            case ADS1115::FullScaleRange::V_0512: return 0.512;
            case ADS1115::FullScaleRange::V_0256: return 0.256;
        }
    }

    uint16_t convertToFixed(const double fsr, const double value) {
        if (value <= -fsr) return MIN_FRACTION;
        if (value >= fsr * ONE_MINUS_EPSILON) return MAX_FRACTION;
        double fraction = round((fsr / value) * BASE_MULTIPLE);
        int16_t intFraction = static_cast<int16_t>(fraction);
        return *reinterpret_cast<uint16_t*>(&intFraction);
    }

    constexpr bool REVERSED = true;
}

uint16_t ADS1115::readConfig() {
    return read_word_data(CONFIG, true) & ~OS;
}

void ADS1115::setMultiplexerConfig(MultiplexerConfig multiplexerConfig) {
    uint16_t config = readConfig();
    applyMultiplexerConfig(config, static_cast<uint16_t>(multiplexerConfig));
    write_word_data(CONFIG, config, REVERSED);
}

void ADS1115::updateFSR(double value) {
    fsr = value;
    setLowThreshold(low_thresh);
    setHighThreshold(high_thresh);
}

void ADS1115::setFullScaleRange(FullScaleRange range) {
    uint16_t config = readConfig();
    applyFullScaleRange(config, static_cast<uint16_t>(range));
    write_word_data(CONFIG, config, REVERSED);
    updateFSR(getFSRValue(range));
}

void ADS1115::setContinuousMode(bool value) {
    uint16_t config = readConfig();
    applyContinuousMode(config, value);
    write_word_data(CONFIG, config, REVERSED);
    continuousMode = value;
}

void ADS1115::setDataRate(DataRate rate) {
    uint16_t config = readConfig();
    applyDataRate(config, static_cast<uint16_t>(rate));
    write_word_data(CONFIG, config, REVERSED);
}

void ADS1115::setup(
    MultiplexerConfig multiplexerConfig,
    FullScaleRange range,
    DataRate rate,
    bool contMode) 
{
    uint16_t config = readConfig();
    applyMultiplexerConfig(config, static_cast<uint16_t>(multiplexerConfig));
    applyFullScaleRange(config, static_cast<uint16_t>(range));
    applyDataRate(config, static_cast<uint16_t>(rate));
    applyContinuousMode(config, contMode);
    write_word_data(CONFIG, config, REVERSED);

    updateFSR(getFSRValue(range));
    continuousMode = contMode;
}

void ADS1115::setComparatorMode(ComparatorMode mode) {
    uint16_t config = readConfig();
    applyComparatorMode(config, mode == ComparatorMode::WINDOW);
    write_word_data(CONFIG, config, REVERSED);
}

void ADS1115::setComparatorPolarity(bool activeHigh) {
    uint16_t config = readConfig();
    applyComparatorPolarity(config, activeHigh);
    write_word_data(CONFIG, config, REVERSED);
}

void ADS1115::setComparatorLatching(bool latching) {
    uint16_t config = readConfig();
    applyComparatorLatching(config, latching);
    write_word_data(CONFIG, config, REVERSED);
}

void ADS1115::setComparatorQueue(ComparatorQueue queue) {
    uint16_t config = readConfig();
    applyComparatorQueue(config, static_cast<uint16_t>(queue));
    write_word_data(CONFIG, config, REVERSED);
}

void ADS1115::setupComparator
    (ComparatorMode mode, ComparatorQueue queue,
    bool activeHigh, bool latching) 
{
    uint16_t config = readConfig();
    applyComparatorMode(config, mode == ComparatorMode::WINDOW);
    applyComparatorQueue(config, static_cast<uint16_t>(queue));
    applyComparatorPolarity(config, activeHigh);
    applyComparatorLatching(config, latching);
    write_word_data(CONFIG, config, REVERSED);
}

double ADS1115::getVoltage() {
    if (!continuousMode) {
        uint16_t config{ 0 };
        auto waitForConversion = [this, &config]() {
            config = read_word_data(CONFIG, REVERSED);
            while (config & OS == 0) {
                usleep(SLEEP_US);
                config = read_word_data(CONFIG, REVERSED);
            }
        };

        waitForConversion();
        // Start conversion by setting OS bit to 1
        write_word_data(CONFIG, config | OS, REVERSED);
        waitForConversion();
    }

    uint16_t value = read_word_data(CONVERSION, REVERSED);
    if (value == MIN_FRACTION) return -fsr;
    if (value == MAX_FRACTION) return fsr * ONE_MINUS_EPSILON;
    int16_t signedValue = *reinterpret_cast<int16_t*>(&value);
    double fraction = static_cast<double>(signedValue) / BASE_MULTIPLE;
    return fraction * fsr;
}

void ADS1115::setLowThreshold(double voltage) {
    write_word_data(LO_THRESH, convertToFixed(fsr, voltage), REVERSED);
    low_thresh = voltage;
}

void ADS1115::setHighThreshold(double voltage) {
    write_word_data(HI_THRESH, convertToFixed(fsr, voltage), REVERSED);
    high_thresh = voltage;
}