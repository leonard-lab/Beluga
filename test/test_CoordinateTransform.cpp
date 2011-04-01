#include <iostream>
#include <fstream>

#include "CalibrationDataFile.h"
#include "CoordinateTransforms.h"

const double water_depth = 2.286;

bool readInput(double* u, double* v, double* d, const char* filename)
{
    try
    {
        std::ifstream ifs(filename);

        ifs >> *u;
        ifs >> *v;
        ifs >> *d;

        if(ifs.fail())
        {
            throw 1;
        }
        
        ifs.close();
    }
    catch(int e)
    {
        std::cerr << "Error reading " << filename << ": " << e << std::endl;
        return false;
    }
    return true;
}

void writeOutputXYZ(double x, double y, double z, const char* filename)
{
    std::ofstream ofs(filename);
    ofs << x << " " << y << " " << z << "\n";
    ofs.close();
}

void writeOutputUV(double u, double v, const char* filename)
{
    std::ofstream ofs(filename);
    ofs << u << " " << v  << "\n";
    ofs.close();
}

const char* const usage = "Usage:  test_CoordinateTransform dir "
           "calibration_file input_file output_file [(un)distort] [-v]\n"
           "  dir = 0: Image+depth to world\n"
           "  dir = 1: World to iamge\n";

const bool I2W = false;
const bool W2I = true;

int main(int argc, char** argv)
{
    if(argc < 5)
    {
        std::cout << usage;
        return 1;
    }

    bool undistort = false;
    if(argc >= 6)
    {
        if(!strncmp(argv[5], "1", MIN(1, strlen(argv[5])))
           || !strncmp(argv[5], "yes", MIN(3, strlen(argv[5]))))
        {
            undistort = true;
        }
    }

    bool verbose = false;
    if(argc == 7)
    {
        if(!strncmp(argv[6], "-v", MIN(3, strlen(argv[6]))))
        {
            verbose = true;
        }
    }
    
    bool dir = I2W;
    if(!strncmp(argv[1], "0", 1))
    {
        dir = I2W;
    }
    else if(!strncmp(argv[1], "1", 1))
    {
        dir = W2I;
    }
    else
    {
        std::cout << usage;
        return 1;
    }
    
    Beluga_CalibrationDataFile inputCal(argv[2]);
    
    if(!inputCal.didLoadOK())
    {
        std::cerr << "Failure to load or validate calibration file "
                  << argv[1] << "\n";
        return 1;
    }
    CalibrationData cal;
    inputCal.getCalibration(&cal);

    CTWithWater ct(cal, water_depth);

    double u = 0;
    double v = 0;
    double d = 0;
    double x = 0;
    double y = 0;
    double z = 0;

    if(dir == I2W)
    {
        if(!readInput(&u, &v, &d, argv[3]))
        {
            return 1;
        }

        if(verbose)
        {
            std::cout << "Read point: " << u << ", " << v
                      << " at depth " << d << std::endl;
        }

        ct.imageAndDepthToWorld(u, v, d, &x, &y, &z, undistort);

        if(verbose)
        {
            std::cout << "Transforms to: " << x << ", " << y
                      << ", " << z << std::endl;
        }

        writeOutputXYZ(x, y, z, argv[4]);
    }
    else
    {
        if(!readInput(&x, &y, &z, argv[3]))
        {
            return 1;
        }

        if(verbose)
        {
            std::cout << "Read world point: " << x << ", " << y
                      << ", " << z << std::endl;
        }

        ct.worldToImage(x, y, z, &u, &v, undistort);

        if(verbose)
        {
            std::cout << "Transforms to: " << u << ", " << v << std::endl;
        }

        writeOutputUV (u, v, argv[4]);
    }
    
    return 0;
}
