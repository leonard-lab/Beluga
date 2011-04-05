#ifndef TRACKEREXTRAS_H
#define TRACKEREXTRAS_H

#include <cv.h>
#include <vector>

void rollHistories(std::vector<double>* X,
                   std::vector<double>* Y,
                   double x,
                   double y,
                   unsigned int N_hist);

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
