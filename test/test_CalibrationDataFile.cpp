#include <iostream>

#include "CalibrationDataFile.h"

int main(int argc, char** argv)
{
    Beluga_CalibrationDataFile testfile("../matlab/calibration/test.dat");

    if(!testfile.didLoadOK())
    {
        std::cerr << "Failure to load or validate file\n";
        return 1;
    }

    CalibrationData d;
    testfile.getCalibration(&d);
    
    std::cout << calibrationDataToString(d);

    return 0;
}
