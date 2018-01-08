#include <GPIOPin.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using std::vector;
using std::string;
using std::ofstream;
using std::ifstream;
using std::stringstream;

namespace {
    constexpr size_t MAX_GPIO_PIN = 27;

    vector<bool>& activePins() {
        static vector<bool> pins(MAX_GPIO_PIN + 1, false);
        return pins;
    }

    const string GPIO_EXPORT_PATH{ "/sys/class/gpio/export" };
    void exportPin(size_t pin) {
        auto& active = activePins();
        if (active[pin]) {
            std::cerr << "Cannot export pin " << pin
                      << " - already active" << std::endl;
        } else {
            ofstream fout{ GPIO_EXPORT_PATH };
            fout << pin; active[pin] = true;
        }
    }

    const string GPIO_UNEXPORT_PATH{ "/sys/class/gpio/unexport" };
    void unexportPin(size_t pin) {
        auto& active = activePins();
        if (!active[pin]) {
            std::cerr << "Cannot unexport pin " << pin
                << " - not active" << std::endl;
        } else {
            ofstream fout{ GPIO_UNEXPORT_PATH };
            fout << pin; active[pin] = false;
        }
    }

    const string GPIO_ROOT{ "/sys/class/gpio/gpio" };
    const string s_direction{ "direction" };
    const string s_value{ "value" };
    string gpioPath(size_t pin, const string& file) {
        stringstream ss;
        ss << GPIO_ROOT << pin << '/' << file;
        return ss.str();
    }
}

GPIOPin::GPIOPin(unsigned char pin, Direction dir) 
    : pin(pin), dir(dir), value(false) 
{
    exportPin(pin);
    setDirection(dir);
}

GPIOPin::~GPIOPin() {
    unexportPin(pin);
}

void GPIOPin::setDirection(Direction dir) {
    ofstream fout{ gpioPath(pin, s_direction) };
    switch (dir) {
        case Direction::INPUT: fout << "in"; break;
        case Direction::OUTPUT: fout << "out"; break;
    }
}

void GPIOPin::setValue(bool val) {
    if (dir != Direction::OUTPUT) {
        std::cerr << "Cannot set value for pin " << pin
                  << " - direction is set to INPUT" << std::endl;
    } else {
        ofstream fout{ gpioPath(pin, s_value) };
        fout << (val ? 1 : 0);
        value = val;
    }
}

bool GPIOPin::getValue() {
    if (dir != Direction::INPUT) {
        return value;
    } else {
        size_t val;
        ifstream fin{ gpioPath(pin, s_value) };
        fin >> val;
        value = (val != 0);
        return value;
    }
}