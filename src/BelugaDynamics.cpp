#include "BelugaDynamics.h"

#include "MT/MT_Core/support/mathsupport.h"

/* This is the state mapping used by the Unscented Kalman Filter
 * (UKF).  I.e.
 * x(t+1) = f(x(t), u(t) (control input), v(t) (process noise)
 *
 * This particular function is based on a model where heading and
 * speed remain constant unless they are changed by "random" noise.
 * In reality the beluga are speeding up, slowing down, and turning as a
 * result of behavioral decisions, but without knowing what those
 * decisions were, the best we can do is model it as noise and guess
 * at the standard deviations.  In this model we don't use u_k, which
 * is fine in terms of the UKF.
 *
 * The k subscript is for time - e.g. x_k = x(kT) where T is the
 * sampling time
 */
void beluga_dynamics(const CvMat* x_k,
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
	double z = cvGetReal2D(x_k, 2, 0);
    double hdg = cvGetReal2D(x_k, 3, 0); /* heading [rad] */
    double spd = cvGetReal2D(x_k, 4, 0); /* speed */

    /* position(t + 1) = position(t) + dT*velocity */
    x += dT*spd*cos(hdg);
    y += dT*spd*sin(hdg);

    /* works just like cvGetReal2D */
    cvSetReal2D(x_kplus1, 0, 0, x);
    cvSetReal2D(x_kplus1, 1, 0, y);
    cvSetReal2D(x_kplus1, 2, 0, z);
    cvSetReal2D(x_kplus1, 3, 0, hdg);
    cvSetReal2D(x_kplus1, 4, 0, fabs(spd));

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
void beluga_measurement(const CvMat* x_k,
                             const CvMat* n_k,
                             CvMat* z_k)
{
	unsigned int nrows = z_k->rows;
	unsigned int nmeas = (nrows-1)/3;
	if( (3*nmeas) != (nrows-1))
	{
		fprintf(stderr, "beluga_measurement error:  "
			"Number of rows in z is incorrect.  Must be 3*n+1.\n");
		return;
	}

	/* measurement should be (x, y, theta) repeated nmeas times then z */
	for(unsigned int i = 0; i < nmeas; i++)
	{
		cvSetReal2D(z_k, 3*i+0, 0, cvGetReal2D(x_k, 0, 0));
		cvSetReal2D(z_k, 3*i+1, 0, cvGetReal2D(x_k, 1, 0));
		cvSetReal2D(z_k, 3*i+2, 0, cvGetReal2D(x_k, 3, 0));
	}
    cvSetReal2D(z_k, nrows-1, 0, cvGetReal2D(x_k, 2, 0));

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
void constrain_state(CvMat* x_k,
                            CvMat* X_p,
							double tank_radius,
							double water_depth)
{
    double x = cvGetReal2D(x_k, 0, 0);
    double y = cvGetReal2D(x_k, 1, 0);
    double z = cvGetReal2D(x_k, 2, 0);
    double hdg = cvGetReal2D(x_k, 3, 0);
    double spd = cvGetReal2D(x_k, 4, 0);

    double x_p = cvGetReal2D(X_p, 0, 0);
    double y_p = cvGetReal2D(X_p, 1, 0);
    double z_p = cvGetReal2D(X_p, 2, 0);
    double hdg_p = cvGetReal2D(X_p, 3, 0);
    double spd_p = cvGetReal2D(X_p, 4, 0);

	if(x*x + y*y > tank_radius)
	{
		double phi = atan2(y, x);
		x = 0.95*tank_radius*cos(phi);
		y = 0.95*tank_radius*sin(phi);
	}

    z = MT_CLAMP(z, 0, water_depth);
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
    if(MT_isnan(z))
    {
        if(!MT_isnan(z_p))
        {
            z = z_p;
        }
        else
        {
            z = water_depth;
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
    cvSetReal2D(x_k, 2, 0, z);
    cvSetReal2D(x_k, 3, 0, hdg);
    cvSetReal2D(x_k, 4, 0, fabs(spd));
}
