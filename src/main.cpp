#include <Hardware.hpp>
#include <i2c.hpp>
#include <unistd.h>
#include <iostream>
#include <string>
#include <iomanip>

void printVector(const std::string& tag, const value3d& vector) {
    std::cout << tag
              << "   X:" << std::setprecision(6) << vector[0]
              << "   Y:" << std::setprecision(6) << vector[1]
              << "   Z:" << std::setprecision(6) << vector[2] << std::endl;
}

void setMirrorPositionManually(Hardware& hw) {
    double pan{ 0 }, tilt{ 0 };
    std::cout << "Enter pan angle: " << std::flush;
    std::cin >> pan;
    std::wcout << "Enter tilt angle: " << std::flush;
    std::cin >> tilt;
    hw.setMirrorPan(pan);
    hw.setMirrorTilt(tilt);

    sleep(5);
    printVector("ACCELER", hw.getAcceleration());
    printVector("COMPASS", hw.getMagnetometer());
}

int main(int argc, char* argv[]) {   
    //i2c::setLogging(true);
    Hardware hw;
    hw.calibratePan();
    /*
    while (true) {
        getchar();
        double voltage = hw.getVoltage();
        std::cout << "V = " << voltage << std::endl;

        getchar();
        setMirrorPositionManually(hw);
    }
    */
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
