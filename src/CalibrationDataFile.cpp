#include "CalibrationDataFile.h"

#include <sstream>
#include <iomanip>

#include "MT/MT_Core/support/filesupport.h"

static const std::string tag_line_start = "Camera calibration - ";
static const std::string tag_line_middle = " rows w/lengths [";
static const unsigned int num_rows_expected = 9;
static const int row_lengths_expected[] = {-1, 2, 2, 1, 5, 3, 3, 3, 3};

static bool readAndValidateLine(FILE* f, int i, double* data)
{
    std::vector<double> row_data = MT_ReadDoublesToEndOfLine(f);

    if(row_data.size() != row_lengths_expected[i])
    {
        fprintf(stderr,
                "CalibrationDataFile read error: Row length mismatch [b, %d]\n",
                i);
        return false;
    }

    for(unsigned int j = 0; j < row_lengths_expected[i]; j++)
    {
        data[j] = row_data[j];
    }

    return true;
}

static bool validateAndRead(FILE* f, CalibrationData* data)
{
    std::string line1 = MT_TextToEndOfLine(f);

    int r = line1.substr(0, tag_line_start.length()).compare(tag_line_start);
    if(r != 0)
    {
        fprintf(stderr, "CalibrationDataFile read error:  Mismatch [a] in head of file\n");
        return false;
    }

    line1 = line1.substr(tag_line_start.length());

    std::stringstream ss;
    ss.str(line1);
    int num_rows;
    std::string v;
    ss >> num_rows;

    if(num_rows != num_rows_expected)
    {
        fprintf(stderr, "CalibrationDataFile read error:  Number of rows mismatch [a]\n");
        return false;
    }

    int nr = ss.tellg();
    line1 = line1.substr(nr);

    r = line1.substr(0, tag_line_middle.length()).compare(tag_line_middle);
    if(r != 0)
    {
        fprintf(stderr, "CalibrationDataFile read error:  Mismatch [b] in head of file\n");
        return false;
    }

    line1 = line1.substr(tag_line_middle.length());
    ss.str(line1);
    for(int i = 0; i < num_rows; i++)
    {
        ss >> nr;
        if(nr != row_lengths_expected[i])
        {
            fprintf(stderr,
                    "CalibrationDataFile read error:  Row length mismatch [a, %d]\n",
                    i);
            return false;
        }
    }

    data->info = line1.substr(((int) ss.tellg()) + 4); /* strip "] - " */

    if(!readAndValidateLine(f, 1, data->f)){return false;};
    if(!readAndValidateLine(f, 2, data->c)){return false;};
    if(!readAndValidateLine(f, 3, &data->alpha)){return false;};
    if(!readAndValidateLine(f, 4, data->k)){return false;};
    if(!readAndValidateLine(f, 5, &(data->R[0]))){return false;};
    if(!readAndValidateLine(f, 6, &(data->R[3]))){return false;};
    if(!readAndValidateLine(f, 7, &(data->R[6]))){return false;};
    if(!readAndValidateLine(f, 8, data->T)){return false;};
    
    return true;
    
}

void copyCalibrationData(CalibrationData* dest, const CalibrationData& src)
{
    dest->info = std::string(src.info);
    memcpy(dest->f, src.f, 2*sizeof(double));
    memcpy(dest->c, src.c, 2*sizeof(double));
    dest->alpha = src.alpha;
    memcpy(dest->k, src.k, 5*sizeof(double));
    memcpy(dest->R, src.R, 9*sizeof(double));
    memcpy(dest->T, src.T, 3*sizeof(double));    
}

std::string calibrationDataToString(const CalibrationData& calibData)
{
    std::string result;
    std::stringstream ss;

    ss << "Camera calibration:\n\t" << calibData.info << std::endl;
    ss << "\tFocal Parameters: " << calibData.f[0] << ", " << calibData.f[1] << std::endl;
    ss << "\tPrincipal Point: " << calibData.c[0] << ", " << calibData.c[1] << std::endl;
    ss << "\tSkew Coefficient: " << calibData.alpha << std::endl;
    ss << "\tRadial Distortion Ceofficients: " <<
        calibData.k[0] << ", " <<
        calibData.k[1] << ", " <<
        calibData.k[2] << ", " <<
        calibData.k[3] << ", " <<
        calibData.k[4] << ", " << std::endl;        
    ss << "\tRotation Matrix: \n\t  [" << 
        calibData.R[0] << " " <<
        calibData.R[1] << " " <<
        calibData.R[2] << "]\n\t  [" <<
        calibData.R[3] << " " <<
        calibData.R[4] << " " <<
        calibData.R[5] << "]\n\t  [" <<
        calibData.R[6] << " " <<
        calibData.R[7] << " " <<
        calibData.R[8] << "]\n";
    ss << "\tTranslation: \n\t  [" <<
       calibData.T[0] << " " <<
       calibData.T[0] << " " <<
        calibData.T[0] << "]\n";        

    return ss.str();
}

Beluga_CalibrationDataFile::Beluga_CalibrationDataFile(const char* filename)
    : m_Data(),
      m_bDidLoadOK(false)
{
    if(!MT_FileIsAvailable(filename, "r"))
    {
        fprintf(stderr, "CalibrationDataFile Error:  Could not load %s", filename);
        return;
    }

    FILE* f = fopen(filename, "r");

    m_bDidLoadOK = validateAndRead(f, &m_Data);

    fclose(f);
    
}

Beluga_CalibrationDataFile::~Beluga_CalibrationDataFile()
{
}

bool Beluga_CalibrationDataFile::didLoadOK() const
{
    return m_bDidLoadOK;
}

bool Beluga_CalibrationDataFile::getCalibration(CalibrationData* calibData)
{
    if(!calibData || !m_bDidLoadOK)
    {
        return false;
    }

    copyCalibrationData(calibData, m_Data);
    
    return true;
}

