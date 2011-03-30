#include "CoordinateTransforms.h"

#include <math.h>

#include <cv.h>

#include "CalibrationDataFile.h"

const double n1 = 1.333;    /* water refractive index */
const double n2 = 1.0;      /* air refractive index */

/* Snell's law equation used to find image of underwater points */
double snellEquation(double rp, double rs, double d, double h)
{
    return (n1*n1*(h*h+rs*rs)-n2*n2*rs*rs)*(rp-rs)*(rp-rs)-n2*n2*d*d*rs*rs;
}

/* derivative of snellEquation with respect to rp, used in
 * solveRsFromRp */
double snellDerivative(double rp, double rs, double d, double h)
{
    return (2.0*n1*n1*rs - 2.0*n2*n2*rs)*(rp-rs)*(rp-rs)
        - 2.0*(n1*n1*(h*h+rs*rs) - n2*n2*rs*rs)*(rp-rs) - 2.0*n2*n2*d*d*rs;
}

/* Uses Newton's method to solve for rS when rP is given
 * Iterates until snellEquation is less than threshold variable, or
 * for a maximum of imax iteration */
double solveRsFromRp(double rp, double d, double h)
{
    const double thresh = 1e-4;
    const unsigned int imax = 10;

    double f;
    double fd;
    double rs = 0.5*rp;
    for(unsigned int i = 0; i < imax; i++)
    {
        f = snellEquation(rp, rs, d, h);
        if(fabs(f) < thresh)
        {
            break;
        }

        fd = snellDerivative(rp, rs, d, h);
        if(fabs(fd) < thresh)
        {
            fd = -1;
        }

        rs -= f/fd;

        if(rs < 0)
        {
            rs = 0;
        }
        if(rs > rp)
        {
            rs = rp;
        }
    }

    return rs;
}

/* solves the Snell's law equation for rP given rS */
double calculateRpFromRs(double rs, double d, double h)
{
    return rs + n2*rs*d/sqrt(n1*n1*(h*h+rs*rs) - n2*n2*rs*rs);
}

CTWithWater::CTWithWater(const CalibrationData& calibrationData, double water_depth)
    : m_dWaterDepth(water_depth)
{
    m_CameraMatrix = cvCreateMat(3, 3, CV_32FC1);
    m_DistCoeffs = cvCreateMat(5, 1, CV_32FC1);
    m_Rv = cvCreateMat(1, 3, CV_32FC1);
    m_R = cvCreateMat(3, 3, CV_32FC1);
    m_T = cvCreateMat(3, 1, CV_32FC1);

    m_CameraWorld = cvCreateMat(3, 1, CV_32FC1);

    m_S = cvCreateMat(3, 1, CV_32FC1);

    m_CameraMatrixNorm = cvCreateMat(3, 3, CV_32FC1);
    m_DistCoeffsNorm = cvCreateMat(5, 1, CV_32FC1);

    setCalibration(calibrationData);
}

CTWithWater::~CTWithWater()
{
    cvReleaseMat(&m_CameraMatrix);
    cvReleaseMat(&m_DistCoeffs);
    cvReleaseMat(&m_Rv);
    cvReleaseMat(&m_R);
    cvReleaseMat(&m_T);

    cvReleaseMat(&m_CameraWorld);

    cvReleaseMat(&m_S);
    
    cvReleaseMat(&m_CameraMatrixNorm);
    cvReleaseMat(&m_DistCoeffsNorm);    
}

void CTWithWater::setCalibration(const CalibrationData& calibrationData)
{
    calibrationDataToOpenCVCalibration(calibrationData,
                                       m_CameraMatrix,
                                       m_DistCoeffs,
                                       m_Rv,
                                       m_T,
                                       m_R);

    /* CameraWorld = -R^T T */
    cvGEMM(m_R, m_T, -1.0, NULL, 0, m_CameraWorld, CV_GEMM_A_T);

    m_dCameraZ = cvGetReal1D(m_CameraWorld, 2);
    m_dCameraHeightAboveWater = m_dCameraZ - m_dWaterDepth;

    cvSetIdentity(m_CameraMatrixNorm);
    /* TODO: These should really be read in from the calibration */
    cvSetReal2D(m_CameraMatrixNorm, 0, 0, 472.0);
    cvSetReal2D(m_CameraMatrixNorm, 1, 1, 472.0);
    cvSetReal2D(m_CameraMatrixNorm, 0, 2, 320.0);
    cvSetReal2D(m_CameraMatrixNorm, 1, 2, 240.0);
    cvZero(m_DistCoeffsNorm);
}

void CTWithWater::worldToImage(double x, double y, double z,
                               double* u, double* v,
                               bool distort)
{
    double d = m_dWaterDepth - z;

    /* d > 0 => point is below surface of water */
    if(d > 0)
    {
        double phi = atan2(y - cvGetReal1D(m_CameraWorld, 1),
                           x - cvGetReal1D(m_CameraWorld, 0));
        double rP = sqrt((x-cvGetReal1D(m_CameraWorld, 0))
                         *(x-cvGetReal1D(m_CameraWorld, 0)) + 
                         (y-cvGetReal1D(m_CameraWorld, 1))
                         *(y-cvGetReal1D(m_CameraWorld, 1)));
        double rS = solveRsFromRp(rP, d, m_dCameraHeightAboveWater);
        cvSetReal1D(m_S, 0, rS*cos(phi));
        cvSetReal1D(m_S, 1, rS*sin(phi));
        cvSetReal1D(m_S, 2, -m_dCameraHeightAboveWater);

        /* S = CW + S */
        cvAdd(m_S, m_CameraWorld, m_S);
    }
    else
    {
        /* point is above water surface and can be used directly */
        cvSetReal1D(m_S, 0, x);
        cvSetReal1D(m_S, 1, y);
        cvSetReal1D(m_S, 2, z);
    }

    /* S = R*S + T */
    cvGEMM(m_R, m_S, 1.0, m_T, 1.0, m_S);

    /* in camera coordinats */
    x = cvGetReal1D(m_S, 0);
    y = cvGetReal1D(m_S, 1);
    z = cvGetReal1D(m_S, 2);

    if(z == 0)
    {
        z = 1.0;
    }

    x = x/z;
    y = y/z;

    if(distort)
    {
        double xx, yy, r2, k1, k2, k3, p1, p2;
        r2 = (x*x + y*y);
        k1 = cvGetReal1D(m_DistCoeffs, 0);
        k2 = cvGetReal1D(m_DistCoeffs, 1);
        p1 = cvGetReal1D(m_DistCoeffs, 2);
        p2 = cvGetReal1D(m_DistCoeffs, 3);
        k3 = cvGetReal1D(m_DistCoeffs, 4);        
        xx = x*(1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2) +
            2.0*p1*x*y + p2*(r2 + 2.0*x*x);
        yy = y*(1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2) +
            p1*(r2 + 2.0*y*y) + 2.0*p2*x*y;
        *u = cvGetReal2D(m_CameraMatrix, 0, 0)*x + cvGetReal2D(m_CameraMatrix, 0, 2);
        *v = cvGetReal2D(m_CameraMatrix, 1, 1)*y + cvGetReal2D(m_CameraMatrix, 1, 2);
    }
    else
    {
        *u = cvGetReal2D(m_CameraMatrixNorm, 0, 0)*x
            + cvGetReal2D(m_CameraMatrixNorm, 0, 2);
        *v = cvGetReal2D(m_CameraMatrixNorm, 1, 1)*y
            + cvGetReal2D(m_CameraMatrixNorm, 1, 2);
    }

}

void CTWithWater::imageAndDepthToWorld(double u, double v, double d,
                                       double* x, double* y, double* z,
                                       bool undistort)
{
    double xx, yy, t;
    CvMat* r = cvCreateMat(3, 1, CV_32FC1);

    if(undistort)
    {
        CvMat* I = cvCreateMat(1, 1, CV_32FC2);
        CvMat* Io = cvCreateMat(1, 1, CV_32FC2);
        cvSet2D(I, 0, 0, cvScalar(u,v));
        
        cvUndistortPoints(I, Io, m_CameraMatrix, m_DistCoeffs, NULL, m_CameraMatrixNorm);
        CvScalar s = cvGet2D(Io, 0, 0);
        
        xx = s.val[0];//cvGetReal1D(Io, 0);
        yy = s.val[1];//cvGetReal1D(Io, 1);
        
        cvReleaseMat(&I);
        cvReleaseMat(&Io);            
    }
    else
    {
        xx = u;
        yy = v;
    }

    xx = (xx - cvGetReal2D(m_CameraMatrixNorm, 0, 2))/cvGetReal2D(m_CameraMatrixNorm, 0, 0);
    yy = (yy - cvGetReal2D(m_CameraMatrixNorm, 1, 2))/cvGetReal2D(m_CameraMatrixNorm, 1, 1);

    cvSetReal1D(r, 0, xx); 
    cvSetReal1D(r, 1, yy);
    cvSetReal1D(r, 2, 1.0);

    /* Rt_(3,:)*r = sum of third column of R times elements of r */
    t = xx*cvGetReal2D(m_R, 0, 2) + yy*cvGetReal2D(m_R, 1, 2) + cvGetReal2D(m_R, 2, 2);
    if(t == 0)
    {
        t = 1.0;
    }

    if(d <= 0)
    {
        /* d<= 0 => above water surface */
        t = (-m_dCameraHeightAboveWater-d)/t;

        /* r = t*R'*r + C */
        cvGEMM(m_R, r, t, m_CameraWorld, 1.0, r, CV_GEMM_A_T);
    }
    else
    {
        /* d > 0 => below water surface */

        t = -m_dCameraHeightAboveWater/t;
        
        /* S = t*R'*r */
        cvGEMM(m_R, r, t, NULL, 0, m_S, CV_GEMM_A_T);

        double Sx = cvGetReal1D(m_S, 0);
        double Sy = cvGetReal1D(m_S, 1);    
        double phi = atan2(Sy, Sx);
        double rS = sqrt(Sx*Sx + Sy*Sy);

        double rP = calculateRpFromRs(rS, d, m_dCameraHeightAboveWater);
        cvSetReal1D(r, 0, rP*cos(phi));
        cvSetReal1D(r, 1, rP*sin(phi));
        cvSetReal1D(r, 2, -m_dCameraHeightAboveWater-d);

        cvAdd(r, m_CameraWorld, r);
    }

    *x = cvGetReal1D(r, 0);
    *y = cvGetReal1D(r, 1);
    *z = cvGetReal1D(r, 2);    
                           
    cvReleaseMat(&r);

}
