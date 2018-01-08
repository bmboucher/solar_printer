#include <i2cDevice.hpp>
#include <Hardware.hpp>
#include <unistd.h>
#include <iostream>

int main(int argc, char* argv[]) {   
    i2c::setLogging(true);
    i2c::softwareReset();

    Hardware hw;
    double pan = -90;
    double inc = 10;
    while (true) {
        hw.setMirrorPan(pan);
        pan += inc;
        if (pan >= 90) inc = -10;
        if (pan <= -90) inc = 10;
        usleep(250000);
        for (double tilt = 0; tilt < 90; tilt += 10) {
            hw.setMirrorTilt(tilt);
            usleep(250000);
        }
        for (double tilt = 80; tilt >= 0; tilt -= 10) {
            hw.setMirrorTilt(tilt);
            usleep(250000);
        }
    }
}
