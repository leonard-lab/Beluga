

double snellEquation(double rp, double rs, double d, double h);
double snellDerivative(double rp, double rs, double d, double h);
double solveRsFromRp(double rp, double d, double h);
double calculateRpFromRs(double rs, double d, double h);

struct CalibrationData;
struct CvMat;

class CTWithWater
{
public:
    CTWithWater(const CalibrationData& calibrationData, double water_depth);
    ~CTWithWater();

    void setCalibration(const CalibrationData& calibrationData);

    void worldToImage(double x, double y, double z,
                      double* u, double* v,
                      bool distort = true);
    
    void imageAndDepthToWorld(double u, double v, double d,
                                           double* x, double* y, double* z,
                                           bool undistort = true);

    /* exposed intentionally: for use in GUI */
    double m_dWaterDepth;

protected:
    CvMat* m_CameraMatrix;
    CvMat* m_DistCoeffs;
    CvMat* m_Rv;
    CvMat* m_R;
    CvMat* m_T;
    CvMat* m_CameraWorld;
    CvMat* m_S;
    CvMat* m_CameraMatrixNorm;
    CvMat* m_DistCoeffsNorm;

    double m_dCameraZ;
    double m_dCameraHeightAboveWater;
    
};
