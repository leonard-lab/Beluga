#ifndef TRACKEREXTRAS_H
#define TRACKEREXTRAS_H

#include <cv.h>
#include <vector>

#include "MT/MT_Core/support/UKF.h"

//void adjustUKFMeasurementSize(MT_UKF_struct* pUKF, unsigned int nmeas);
void adjustRMatrixAndZForMeasurementSize(CvMat*& R, CvMat*& z, unsigned int nmeas);

void rollHistories(std::vector<double>* X,
                   std::vector<double>* Y,
                   double x,
                   double y,
                   unsigned int N_hist);

double rectifyAngleMeasurement(double meas,
							   const std::vector<double>& hist_X,
							   const std::vector<double>& hist_Y,
							   unsigned int N_hist,
							   double angle_prev);

bool CvMatIsOk(const CvMat* M, double max_val = 1e10);

CvRect searchRectAtPointWithSize(double x_center, double y_center, double size);

bool cvRectsIntersect(const CvRect& a, const CvRect& b);
bool pointInCvRect(double px, double py, const CvRect& r);

CvRect unionOfCvRects(const CvRect& a, const CvRect&b);

std::vector<unsigned int> unionOfIndexSets(const std::vector<unsigned int>& a,
										   const std::vector<unsigned int>& b);

void combineSearchAreas(std::vector<CvRect>* searchAreas, 
						std::vector<std::vector<unsigned int> >* searchIndexes);



#endif // TRACKEREXTRAS_H
