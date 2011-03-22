// compile:  g++ OpenCVCalib.cpp `pkg-config opencv --cflags --libs` -o OpenCVCalib

#include <stdio.h>
#include <iostream>
#include <fstream>

#include <cv.h>

/* this function just reads the points from an ascii file
 * and stuffs them into OpenCV matrix objects */
void readPoints(const char* filename,
                CvMat** worldPoints,
                CvMat** imagePoints,
                int* numPoints)
{
    ifstream ptsFile(filename);

    if(!ptsFile.is_open())
    {
        std::cerr << "Error opening " << filename << ".\n";
        return;
    }

    std::vector<double> img;
    std::vector<double> w;
    double ix, iy, wx, wy, wz;
    int i = 0;


    while(!ptsFile.eof())        
    {
        ptsFile >> ix >> iy >> wx >> wy >> wz;
        img.push_back(ix);
        img.push_back(480 - iy);
        w.push_back(wx);
        w.push_back(wy);
        w.push_back(wz);
        i++;
    }

    /* last line reads twice (?) */
    i--;
    img.resize(img.size()-2);
    w.resize(w.size()-3);

    *numPoints = i;
    std::cout << "Found " << img.size()/2 << " points, i = " << i << ".\n";

    *worldPoints = cvCreateMat(*numPoints, 3, CV_32FC1);
    *imagePoints = cvCreateMat(*numPoints, 2, CV_32FC1);

    for(int i = 0; i < *numPoints; i++)
    {
        cvSetReal2D(*worldPoints, i, 0, w[3*i + 0]);
        cvSetReal2D(*worldPoints, i, 1, w[3*i + 1]);
        cvSetReal2D(*worldPoints, i, 2, w[3*i + 2]);
        cvSetReal2D(*imagePoints, i, 0, img[2*i + 0]);
        cvSetReal2D(*imagePoints, i, 1, img[2*i + 1]);        
    }
    
}

/* This function is just a pretty-printer for the results */
void displayResults(CvMat* c, CvMat* k, CvMat* Rv, CvMat* R, CvMat* T)
{
    if(c)
    {
        printf("Camera Matrix:\n");
        printf("[%.4f %.4f %.4f]\n", cvGetReal2D(c, 0, 0),
               cvGetReal2D(c, 0, 1), cvGetReal2D(c, 0, 2));
        printf("[%.4f %.4f %.4f]\n", cvGetReal2D(c, 1, 0),
               cvGetReal2D(c, 1, 1), cvGetReal2D(c, 1, 2));
        printf("[%.4f %.4f %.4f]\n", cvGetReal2D(c, 2, 0),
               cvGetReal2D(c, 2, 1), cvGetReal2D(c, 2, 2));
        printf("\n");
    }

    if(k)
    {
        printf("Distortion:\n");
        printf("[%.4f %.4f %.4f %.4f %.4f]\n\n",
               cvGetReal1D(k, 0), cvGetReal1D(k, 1), cvGetReal1D(k, 2),
               cvGetReal1D(k, 3), cvGetReal1D(k, 4));
    }

    if(Rv)
    {
        printf("Rotation v:\n");
        printf("[%.4f %.4f %.4f]\n\n",
               cvGetReal1D(Rv, 0), cvGetReal1D(Rv, 1), cvGetReal1D(Rv, 2));
    }    
    
    if(R)
    {
        printf("Rotation Matrix:\n");
        printf("[%.4f %.4f %.4f]\n", cvGetReal2D(R, 0, 0),
               cvGetReal2D(R, 0, 1), cvGetReal2D(R, 0, 2));
        printf("[%.4f %.4f %.4f]\n", cvGetReal2D(R, 1, 0),
               cvGetReal2D(R, 1, 1), cvGetReal2D(R, 1, 2));
        printf("[%.4f %.4f %.4f]\n", cvGetReal2D(R, 2, 0),
               cvGetReal2D(R, 2, 1), cvGetReal2D(R, 2, 2));
        printf("\n");
    }

    if(T)
    {
        printf("Translation^T:\n");
        printf("[%.4f %.4f %.4f]\n\n",
               cvGetReal1D(T, 0), cvGetReal1D(T, 1), cvGetReal1D(T, 2));
    }
}

int main(int argc, char** argv)
{
    
    if(argc < 2)
    {
        std::cout << "Usage:  " << argv[0] << " datafile\n";
        return -1;
    }

    CvMat* worldPoints;
    CvMat* imagePoints;
    int numPoints = 0;

    /* load the points from an ascii file, specified on the command line */
    readPoints(argv[1], &worldPoints, &imagePoints, &numPoints);

    /* bail if there aren't at least 4 points */
    if(numPoints < 4)
    {
        return -1;
    }

    /***************** Set up ******************/
    /* Here we just set up the necessary OpenCV
     * Matrix structures */

    /* Matrix allocations */
    CvMat* cameraMatrix = cvCreateMat(3, 3, CV_32FC1);
    CvMat* distCoeffs = cvCreateMat(5, 1, CV_32FC1);
    CvMat* Rv = cvCreateMat(1, 3, CV_32FC1);
    CvMat* R = cvCreateMat(3, 3, CV_32FC1);
    CvMat* T = cvCreateMat(1, 3, CV_32FC1);
    CvMat* k = cvCreateMat(5, 1, CV_32FC1);

    /* Camera matrix */
    /* I think that since we already did the undistortion before
     * finding feature points, we want to use "normal" numbers here:
     * fx = fy = 3.5mm/7.4um (focal length of lens in terms of the
     *                         size of one pixel on the CCD sensor)
     * cx = 320, cy = 240 (coordinates of center pixel)
     */
    double fx = 472;
    double fy = 472;
    double cx = 320;
    double cy = 240;

    cvSetIdentity(cameraMatrix);
    cvSetReal2D(cameraMatrix, 0, 0, fx);
    cvSetReal2D(cameraMatrix, 1, 1, fy);
    cvSetReal2D(cameraMatrix, 0, 2, cx);
    cvSetReal2D(cameraMatrix, 1, 2, cy);

    /* Distortion parameters - all zero, since
     *   we're already accounting for that */
    cvZero(k);    

    /**************** Calibration ***************/

    cvFindExtrinsicCameraParams2(worldPoints,
                                 imagePoints,
                                 cameraMatrix,
                                 k,
                                 Rv,
                                 T);

    /******************** Output ******************/

    /* The output of the above is a vectorized version of
     * the rotation parameters - this converts it back to a
     * rotation matrix */
    cvRodrigues2(Rv, R);
    
    displayResults(cameraMatrix, k, Rv, R, T);
    
    return 0;
}
