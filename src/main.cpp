#include <Hardware.hpp>
#include <unistd.h>
#include <iostream>

void setMirrorPositionManually(Hardware& hw) {
    double pan{ 0 }, tilt{ 0 };
    std::cout << "Enter pan angle: " << std::flush;
    std::cin >> pan;
    std::wcout << "Enter tilt angle: " << std::flush;
    std::wcin >> tilt;
    hw.setMirrorPan(pan);
    hw.setMirrorTilt(tilt);
}

int main(int argc, char* argv[]) {   
    Hardware hw;
    double pan = -90;
    double inc = 10;
    while (true) {
        setMirrorPositionManually(hw);

        /*
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
        */
    }
}
