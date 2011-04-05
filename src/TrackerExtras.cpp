#include "TrackerExtras.h"

#include "MT/MT_Core/support/mathsupport.h"

/* This is the state mapping used by the Unscented Kalman Filter
 * (UKF).  I.e.
 * x(t+1) = f(x(t), u(t) (control input), v(t) (process noise)
 *
 * This particular function is based on a model where heading and
 * speed remain constant unless they are changed by "random" noise.
 * In reality the fish are speeding up, slowing down, and turning as a
 * result of behavioral decisions, but without knowing what those
 * decisions were, the best we can do is model it as noise and guess
 * at the standard deviations.  In this model we don't use u_k, which
 * is fine in terms of the UKF.
 *
 * The k subscript is for time - e.g. x_k = x(kT) where T is the
 * sampling time
 */
static void fish_dynamics(const CvMat* x_k,
                          const CvMat* u_k,
                          const CvMat* v_k,
                          CvMat* x_kplus1)
{
    /* Assuming that the time-step is one frame rather than e.g.
     * 33 msec - I take the actual time step into account in
     * analysis. */
    double dT = 1.0;

    /* cvGetReal2D is useful to get an element of a 2D matrix
     * cvGetReal2D(x_k, i, j) gets the (i,k)^th element
     * Note that x_k is a 4x1 matrix here */
    double x = cvGetReal2D(x_k, 0, 0);
    double y = cvGetReal2D(x_k, 1, 0);
    double hdg = cvGetReal2D(x_k, 2, 0); /* heading [rad] */
    double spd = cvGetReal2D(x_k, 3, 0); /* speed */

    /* position(t + 1) = position(t) + dT*velocity */
    x += dT*spd*cos(hdg);
    y += dT*spd*sin(hdg);

    /* works just like cvGetReal2D */
    cvSetReal2D(x_kplus1, 0, 0, x);
    cvSetReal2D(x_kplus1, 1, 0, y);
    cvSetReal2D(x_kplus1, 2, 0, hdg);
    cvSetReal2D(x_kplus1, 3, 0, fabs(spd));

    /* this allows v_k to be a NULL pointer, in which case
     * this step is skipped */
    if(v_k)
    {
        /* simple additive noise: x(t+1) <- x(t+1) + noise */
        cvAdd(x_kplus1, v_k, x_kplus1);
    }
    
}

/* This is the measurement mapping used by the UKF, i.e.
 * z(t) = h(x(t), n(t))
 *
 * In this case, z is a vector with (x,y) position and heading and
 * noise is additive. */
static void fish_measurement(const CvMat* x_k,
                             const CvMat* n_k,
                             CvMat* z_k)
{
    cvSetReal2D(z_k, 0, 0, cvGetReal2D(x_k, 0, 0));
    cvSetReal2D(z_k, 1, 0, cvGetReal2D(x_k, 1, 0));
    cvSetReal2D(z_k, 2, 0, cvGetReal2D(x_k, 2, 0));

    /* as above, skip this step when n_k is null */
    if(n_k)
    {
        cvAdd(z_k, n_k, z_k);
    }
}

/* Using this function below to constrain the state estimate.  The
 * constraint is applied after the UKF correction step.
 *
 * Constraints applied:
 *   0 <= x <= frame (image) width
 *   0 <= y <= frame height
 *   0 <= speed <= 100 (px/frame)
 *   If x, y, heading, or speed are NaN, then
 *      a) if the corresponding estimate is valid, that number is used
 *      b) if the estimate is also NaN, x, y, and heading are set to
 *          0, speed is set to 0.1 
 */
static void constrain_state(CvMat* x_k,
                            CvMat* X_p,
                            double xmax,
							double ymax)
{
    double x = cvGetReal2D(x_k, 0, 0);
    double y = cvGetReal2D(x_k, 1, 0);
    double hdg = cvGetReal2D(x_k, 2, 0);
    double spd = cvGetReal2D(x_k, 3, 0);

    double x_p = cvGetReal2D(X_p, 0, 0);
    double y_p = cvGetReal2D(X_p, 1, 0);
    double hdg_p = cvGetReal2D(X_p, 2, 0);
    double spd_p = cvGetReal2D(X_p, 3, 0);

    /* MT_CLAMP(x, a, b) =
     *    x, if a <= x <= b,
     *    a, if x < a,
     *    b, if x > b
     */
    x = MT_CLAMP(x, 0, xmax);
    y = MT_CLAMP(y, 0, ymax);
    spd = MT_CLAMP(spd, 0, 100);

    /* MT_isnan(x) returns true if x is NaN */
    if(MT_isnan(x))
    {
        if(!MT_isnan(x_p))
        {
            x = x_p;
        }
        else
        {
            x = 0;
        }
    }
    if(MT_isnan(y))
    {
        if(!MT_isnan(y_p))
        {
            y = y_p;
        }
        else
        {
            y = 0;
        }
    }
    if(MT_isnan(hdg))
    {
        if(!MT_isnan(hdg_p))
        {
            hdg = hdg_p;
        }
        else
        {
            hdg = 0;
        }
    }
    if(MT_isnan(spd))
    {
        if(!MT_isnan(spd_p))
        {
            spd = spd_p;
        }
        else
        {
            spd = 0.1;            
        }
    }
    
    cvSetReal2D(x_k, 0, 0, x);
    cvSetReal2D(x_k, 1, 0, y);
    cvSetReal2D(x_k, 2, 0, hdg);
    cvSetReal2D(x_k, 3, 0, fabs(spd));
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
	return cvRect(x_center-0.5*size, y_center-0.5*size, size, size);
}

bool cvRectsIntersect(const CvRect& a, const CvRect& b)
{
	return a.x < (b.x+b.width) && (a.x+a.width) > b.x &&
		a.y < (b.y+b.height) && (a.y+a.height) > b.y;
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
