#ifndef TRACKEREXTRAS_H
#define TRACKEREXTRAS_H

#include <cv.h>
#include <vector>

static void fish_dynamics(const CvMat* x_k,
                          const CvMat* u_k,
                          const CvMat* v_k,
                          CvMat* x_kplus1);

static void fish_measurement(const CvMat* x_k,
                             const CvMat* n_k,
                             CvMat* z_k);

static void constrain_state(CvMat* x_k,
                            CvMat* X_p,
                            double xmax,
							double ymax);

void rollHistories(std::vector<double>* X,
                   std::vector<double>* Y,
                   double x,
                   double y,
                   unsigned int N_hist);

bool CvMatIsOk(const CvMat* M, double max_val = 1e10);

CvRect searchRectAtPointWithSize(double x_center, double y_center, double size);

bool cvRectsIntersect(const CvRect& a, const CvRect& b);

CvRect unionOfCvRects(const CvRect& a, const CvRect&b);

std::vector<unsigned int> unionOfIndexSets(const std::vector<unsigned int>& a,
										   const std::vector<unsigned int>& b);

void combineSearchAreas(std::vector<CvRect>* searchAreas, 
						std::vector<std::vector<unsigned int> >* searchIndexes);



#endif // TRACKEREXTRAS_H
