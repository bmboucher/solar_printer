#include <Hardware.hpp>
#include <i2c.hpp>
#include <unistd.h>
#include <iostream>

void setMirrorPositionManually(Hardware& hw) {
    double pan{ 0 }, tilt{ 0 };
    std::cout << "Enter pan angle: " << std::flush;
    std::cin >> pan;
    std::wcout << "Enter tilt angle: " << std::flush;
    std::cin >> tilt;
    hw.setMirrorPan(pan);
    hw.setMirrorTilt(tilt);
}

int main(int argc, char* argv[]) {   
    i2c::setLogging(true);
    Hardware hw;

    while (true) {
        getchar();
        std::cout << "V = " << hw.getVoltage() << std::endl;
    }
    /*
    double pan = -90;
    double inc = 0.5;
    hw.setMirrorTilt(45);
    while (true) {
        //setMirrorPositionManually(hw);

        
        hw.setMirrorPan(pan);
        pan += inc;
        if (pan >= 90 || pan <= -90) inc = -inc;
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
    */
}
