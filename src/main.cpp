#include <i2cDevice.hpp>
#include <ServoController.hpp>
#include <unistd.h>
#include <iostream>

int main(int argc, char* argv[]) {   
    i2c::setLogging(true);
    i2c::softwareReset();

    ServoController servo;
    double pulse_ms;
    while (true) {
        std::cout << "Enter pulse ms: " << std::flush;
        std::cin >> pulse_ms;
        servo.setPulseWidth(1, 0, pulse_ms);
    }
    /*
    Hardware hw;
    double pan = -90;
    double inc = 10;
    while (true) {
        hw.setMirrorPan(pan);
        pan += inc;
        if (pan >= 90) inc = -10;
        if (pan <= -90) inc = 10;
        sleep(1);
    }
    */
}
