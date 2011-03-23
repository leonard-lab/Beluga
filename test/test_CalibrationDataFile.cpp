#include <iostream>

#include "CalibrationDataFile.h"

int main(int argc, char** argv)
{
    Beluga_CalibrationDataFile testfile("../matlab/calibration/test.dat");
    Beluga_CalibrationDataFile testOutFile("testOut.dat", CALIB_WRITE);

    if(!testfile.didLoadOK())
    {
        std::cerr << "Failure to load or validate file\n";
        return 1;
    }

    CalibrationData d;
    testfile.getCalibration(&d);
    
    std::cout << calibrationDataToString(d);

    if(!testOutFile.didLoadOK())
    {
        std::cerr << "Failure to load output file\n";
        return 1;
    }

    d.info = std::string("WELL HELLO");
    testOutFile.writeCalibration(d);

    Beluga_CalibrationDataFile testInFile("testOut.dat");
    if(!testInFile.didLoadOK())
    {
        std::cerr << "Failure to load or parse second file\n";
        return 1;
    }

    CalibrationData d2;
    testInFile.getCalibration(&d2);

    std::cout << calibrationDataToString(d2);

    return 0;
}
