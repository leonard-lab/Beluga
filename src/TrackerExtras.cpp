#include "TrackerExtras.h"

#include "MT/MT_Core/support/mathsupport.h"

/*void adjustUKFMeasurementSize(MT_UKF_struct* pUKF, unsigned int nmeas)
{
	pUKF->m = nmeas;
	if(pUKF->z)
	{
		cvReleaseMat(&(pUKF->z));
	}
	pUKF->z = cvCreateMat(pUKF->m, 1, CV_64FC1);
}*/

void adjustRMatrixAndZForMeasurementSize(CvMat** R, CvMat** z, unsigned int nmeas)
{
	unsigned int nrows_prev = (*R)->rows;
	unsigned int nrows_z_prev = (*z)->rows;
	if(nrows_prev < 4)
	{
		fprintf(stderr, "adjustRMatrixForMeasurementSize Error:  Existing R is too small.\n");
		return;
	}

	/* x, y, th each meas + z */
	unsigned int nrows_now = 3*nmeas + 1;
	/* same number of measurements -> do nothing */
	if(nrows_now == nrows_prev && nrows_now == nrows_z_prev)
	{
		return;
	}

	printf("adjR a  %d %d\n", nrows_prev, nrows_now);
	double sx = cvGetReal2D(*R, 0, 0);
	double sy = cvGetReal2D(*R, 1, 1);
	double sth = cvGetReal2D(*R, 2, 2);
	double sz = cvGetReal2D(*R, nrows_prev-1, nrows_prev-1);

	printf("adjR b\n");
	cvReleaseMat(R);
	cvReleaseMat(z);
	printf("adjR c\n");
	*R = cvCreateMat(nrows_now, nrows_now, CV_64FC1);
	*z = cvCreateMat(nrows_now, 1, CV_64FC1);
	printf("adjR d\n");
	cvZero(*R);
	cvZero(*z);
	printf("adjR e\n");
	for(unsigned int i = 0; i < nmeas; i++)
	{
		cvSetReal2D(*R, i*3 + 0, i*3 + 0, sx);
		cvSetReal2D(*R, i*3 + 1, i*3 + 1, sy);
		cvSetReal2D(*R, i*3 + 0, i*3 + 0, sth);
	}
	printf("adjR f\n");
	cvSetReal2D(*R, nrows_now-1, nrows_now-1, sz);
}

/* helper function.  Basic FIFO buffer with N_hist entries
 *
 * Given new values for x and y, do
 * if size of X < N_hist,
 *    X = [existing values.... x]
 * else (have N_hist values)
 *    drop the first value, shift all of the other values forward
 *     (i.e. X[k] = X[k+1]), and set the last value to x
 *
 * The same is done for Y. */
void rollHistories(std::vector<double>* X,
                   std::vector<double>* Y,
                   double x,
                   double y,
                   unsigned int N_hist)
{
    if(X->size() < N_hist)
    {
        X->push_back(x);
        Y->push_back(y);
    }
    else
    {
        for(unsigned int i = 0; i < X->size()-1; i++)
        {
            X->at(i) = X->at(i+1);
            Y->at(i) = Y->at(i+1);
        }
        X->at(X->size()-1) = x;
        Y->at(Y->size()-1) = y;
    }
}

double rectifyAngleMeasurement(double meas,
							   const std::vector<double>& hist_X,
							   const std::vector<double>& hist_Y,
							   unsigned int N_hist,
							   double angle_prev)
{
	/* Angle Madness
	*
	* Getting the orientation angle is one of the trickiest
	* parts of tracking.  There's a couple reasons for this.
	*
	* 1. Measurements of orientation are
             *        highly prone to pointing in the opposite
             *        direction (confusing the head for the tail).
             *
             * 2. Orientation measurements will be fixed in a range of
			 *        [-\pi, \pi] or [0, 2\pi], etc, which creates
             *        discontinuities in the measurement.  That is,
             *        suppose your object is moving to the left in the
             *        image plane, so its heading is hovering right
             *        around \pi.  The measurement could go, in one
             *        time step, from \pi - 0.001 to -\pi + 0.001.
             *        This is a bad thing.
             *
             * Here's how we handle these problems.
             *
             * 1.   A) By default, we trust the orientation measurement.
             *            It uses an algorithm that should be able to
             *            distinguish the head from the tail.
             *      B) We keep a history of past positions.  If the
             *            object has moved far enough in the last few
             *            time steps, we use the use the displacement
             *            vector as a cue.  Otherwise we use the last
             *            known orientation estimate as a cue.
             *      C) The orientation measurement is compared to the
             *            cue.  If the two are similar (see below),
             *            the measurement is used as-is.  If they are
             *            not similar, 180 degrees is subtracted from
             *            the measurement.
             *
             * 2.  A) A shortest-arc difference between the
             *            measurement and last known orientation is
             *            calculated.
             *     B) Due to the way we calculate
             *            the orientation measurement, this difference
             *            is most likely less than 90 degrees in
             *            magnitude.
             *     C) The actual measurement given
             *            to the UKF is the current orientation
             *            estimate plus the calculated difference.
             * 
             */
            double th;
            bool a = false;
			double th_meas;
            if(hist_X.size() == N_hist)
            {
                double dx = hist_X.at(N_hist-1) -
                    hist_X.at(0);
                double dy = hist_Y.at(N_hist-1) -
                    hist_Y.at(0);

                double min_move = 0.1;

                /* if the object has moved far enough */
                if(dx*dx + dy*dy > min_move*min_move)
                {
                    /* negative sign accounts for screen coordinates */
                    th = atan2(-dy,dx);
                    a = true;
                }
            }

            /* if we don't have enough history or didn't move far
               enough, use the previous orientation */
            if(!a)
            {
                th = angle_prev;
            }

            /* this is how we determine if the two angles are "close"
               - the magnitude of the order parameter, which is
               equivalent to the length of the average phasor, i.e.
               pth = 0.5*|e^{i*th} + e^{i*orientation measurement}|
            */
            double pth = fabs(
                cos(0.5*(th - MT_DEG2RAD*meas))
                );

            /* sqrt(2)/2 is what we'd get if the angles differ by 90
               degrees */
            if(pth < 0.7)  /* a little less than sqrt(2)/2 */
            {
                /* looks like the orientation is opposite of what it
                   should be, so subtract 180 degrees. */
                th_meas = meas - 180.0;
            }
			else
			{
				th_meas = meas;
			}

            /* taking asin(sin(angle)) ensures that |angle| < pi,
             * i.e. we get the shortest-arc distance  */
            double dth = asin(sin(MT_DEG2RAD*th_meas - meas));

			return angle_prev + dth;
}

/* helper function.  Returns false if any element of M is either NaN
 * or larger in magnitude than max_val */
bool CvMatIsOk(const CvMat* M, double max_val)
{
    double v;
    for(int i = 0; i < M->rows; i++)
    {
        for(int j = 0; j < M->cols; j++)
        {
            v = cvGetReal2D(M, i, j);
            if(MT_isnan(v) || fabs(v) > max_val)
            {
                return false;
            }
        }
    }
    return true;
}

CvRect searchRectAtPointWithSize(double x_center, double y_center, double size)
{
	return cvRect((int)(x_center-0.5*size), (int)(y_center-0.5*size), (int) size, (int)size);
}

bool cvRectsIntersect(const CvRect& a, const CvRect& b)
{
	return a.x < (b.x+b.width) && (a.x+a.width) > b.x &&
		a.y < (b.y+b.height) && (a.y+a.height) > b.y;
}

bool pointInCvRect(double px, double py, const CvRect& r)
{
	return (px >= r.x) && (px <= (r.x+r.width)) 
		&& (py >= r.y) && (py <= (r.y+r.height));
}

CvRect unionOfCvRects(const CvRect& a, const CvRect&b)
{
	int xmin = MT_MIN(a.x, b.x);
	int xmax = MT_MAX(a.x+a.width, b.x+b.width);
	int ymin = MT_MIN(a.y, b.y);
	int ymax = MT_MAX(a.y+a.height, b.y+b.height);

	return cvRect(xmin, ymin, xmax-xmin, ymax-ymin);
}

std::vector<unsigned int> unionOfIndexSets(const std::vector<unsigned int>& a,
										   const std::vector<unsigned int>& b)
{
	std::vector<unsigned int> result;
	result.reserve(a.size()+b.size());
	result.insert(result.end(), a.begin(), a.end());
	result.insert(result.end(), b.begin(), b.end());
	return result;
}

void combineSearchAreas(std::vector<CvRect>* searchAreas, 
						std::vector<std::vector<unsigned int> >* searchIndexes)
{
	if(searchAreas->size() <= 1)
	{
		return;
	}

	std::vector<CvRect> inAreas = *searchAreas;
	std::vector<std::vector<unsigned int> > inIndexes = *searchIndexes;

	std::vector<CvRect> outAreas;
	std::vector<std::vector<unsigned int> > outIndexes;
	int num_joined = 0;

	do
	{
		outAreas.resize(0);
		outIndexes.resize(0);
		num_joined = 0;

		for(unsigned int i = 0; i < inAreas.size(); i++)
		{
			for(unsigned int j = i+1; j < inAreas.size(); j++)
			{
				if(cvRectsIntersect(inAreas[i], inAreas[j]))
				{
					num_joined++;
					outAreas.push_back(unionOfCvRects(inAreas[i], inAreas[j]));
					outIndexes.push_back(unionOfIndexSets(inIndexes[i], inIndexes[j]));
				}
				else
				{
					outAreas.push_back(inAreas[i]);
					outIndexes.push_back(inIndexes[i]);
				}
			}
		}

		inAreas = outAreas;
		inIndexes = outIndexes;
	} while(num_joined > 0);

	*searchAreas = inAreas;
	*searchIndexes = inIndexes;

}
