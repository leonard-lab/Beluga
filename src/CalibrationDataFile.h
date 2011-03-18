#ifndef CALIBRATIONDATAFILE_H
#define CALIBRATIONDATAFILE_H

#include <string>

typedef struct CalibrationData
{
    std::string info;
    double f[2];
    double c[2];
    double alpha;
    double k[5];
    double R[9];
    double T[3];

    CalibrationData()
    {
        info = std::string("N/A");
        f[0] = f[1] = 0;
        c[0] = c[1] = 0;
        alpha = 0;
        k[0] = k[1] = k[2] = 0;
        R[0] = R[1] = R[2] =
            R[3] = R[4] = R[5] =
            R[6] = R[7] = R[8] = 0;
        T[0] = T[1] = T[2] = 0;
    }    
} CalibrationData;

void copyCalibrationData(CalibrationData* dest, const CalibrationData& src);

std::string calibrationDataToString(const CalibrationData& calibData);

class Beluga_CalibrationDataFile
{
public:
    Beluga_CalibrationDataFile(const char* filename);
    ~Beluga_CalibrationDataFile();

    bool didLoadOK() const;
    bool getCalibration(CalibrationData* calibData);

private:
    bool m_bDidLoadOK;
    CalibrationData m_Data;

};

#endif // CALIBRATIONDATAFILE_H
