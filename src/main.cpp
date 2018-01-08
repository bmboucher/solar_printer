#include <i2cDevice.hpp>
#include <Hardware.hpp>
#include <unistd.h>

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
        sleep(1);
    }
}
