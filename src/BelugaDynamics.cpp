#include "BelugaDynamics.h"

#include "BelugaConstants.h"

#include "MT/MT_Core/support/mathsupport.h"

double BelugaDynamicsParameters::m_dDt = 1.0;
double BelugaDynamicsParameters::m_dWaterDepth = DEFAULT_WATER_DEPTH;

double check_nan(double value, double predicted, double default_val)
{
    double value_out = value;
    if(MT_isnan(value))
    {
        if(MT_isnan(predicted))
        {
            value_out = default_val;
        }
        else
        {
            value_out = predicted;
        }
    }
    return value_out;
}

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
    // TODO: We need the RIGHT dt
    double dT = BelugaDynamicsParameters::m_dDt;

    double x     = cvGetReal2D(x_k, BELUGA_STATE_X, 0);
    double y     = cvGetReal2D(x_k, BELUGA_STATE_Y, 0);
    double z     = cvGetReal2D(x_k, BELUGA_STATE_Z, 0);
    double zdot  = cvGetReal2D(x_k, BELUGA_STATE_ZDOT, 0);
    double v     = cvGetReal2D(x_k, BELUGA_STATE_SPEED, 0);
    double theta = cvGetReal2D(x_k, BELUGA_STATE_THETA, 0);
    double omega = cvGetReal2D(x_k, BELUGA_STATE_OMEGA, 0);

    double u_z = 0;
    double u_h = 0;
    double u_steer = 0;
    if(u_k)
    {
        u_z		= cvGetReal2D(u_k, BELUGA_INPUT_VERTICAL_SPEED, 0);
		u_h		= cvGetReal2D(u_k, BELUGA_INPUT_FORWARD_SPEED, 0);
		u_steer = cvGetReal2D(u_k, BELUGA_INPUT_STEERING, 0);
    }

    /* horizontal dynamics */
    x += dT*v*cos(theta);
    y += dT*v*sin(theta);

    /* vdot dynamics */
    double vdot = (K_t*u_h - K_d1*v)/m_eff;
    v += dT*vdot;

    /* steering dynamics */
    theta += dT*omega;

    double omegadot = (r_1*u_steer*K_t*u_h - K_omega*omega)/J;
    omega += dT*omegadot;

    /* vertical dynamics */
    z += dT*zdot;
    double u_up = 0;
    double u_down = 0;
    // MAYBE: is this sign right?
    if(u_h < 0)
    {
        u_up = u_h;
        u_down = 0;
    }
    else
    {
        u_down = u_h;
        u_up = 0;
    }
    double v_vert_inv = 1.0/(fabs(zdot) + v_off);
    double zddot = (1/m_eff)*(eta_up*v_vert_inv*(u_up*u_up + k_vp*u_up)
                              + eta_down*v_vert_inv*(u_down + k_vp*u_down)
                              - k_d*fabs(zdot)*zdot
                              + k_teth*(z_off - z));
    zdot += dT*zddot;
    
    /* works just like cvGetReal2D */
    cvSetReal2D(x_kplus1, BELUGA_STATE_X, 0, x);
	cvSetReal2D(x_kplus1, BELUGA_STATE_Y, 0, y);
	cvSetReal2D(x_kplus1, BELUGA_STATE_Z, 0, z);
	cvSetReal2D(x_kplus1, BELUGA_STATE_ZDOT, 0, zdot);
	cvSetReal2D(x_kplus1, BELUGA_STATE_SPEED, 0, v);
	cvSetReal2D(x_kplus1, BELUGA_STATE_THETA, 0, theta);
	cvSetReal2D(x_kplus1, BELUGA_STATE_OMEGA, 0, omega);

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
        double x = cvGetReal2D(x_k, BELUGA_STATE_X, 0);
        double y = cvGetReal2D(x_k, BELUGA_STATE_Y, 0);
        double theta = cvGetReal2D(x_k, BELUGA_STATE_THETA, 0);

        double x_noise = 0;
        double y_noise = 0;
        double theta_noise = 0;
        if(n_k)
        {
            x_noise = cvGetReal2D(n_k, (BELUGA_NUM_MEAS-1)*i + BELUGA_MEAS_X, 0);
            y_noise = cvGetReal2D(n_k, (BELUGA_NUM_MEAS-1)*i + BELUGA_MEAS_Y, 0);
            theta_noise = cvGetReal2D(n_k, (BELUGA_NUM_MEAS-1)*i + BELUGA_MEAS_THETA, 0);
        }

        x += x_noise;
        y += y_noise;
        theta += theta_noise;
        
		cvSetReal2D(z_k, (BELUGA_NUM_MEAS-1)*i+BELUGA_MEAS_X,     0, x);
		cvSetReal2D(z_k, (BELUGA_NUM_MEAS-1)*i+BELUGA_MEAS_Y,     0, y);
		cvSetReal2D(z_k, (BELUGA_NUM_MEAS-1)*i+BELUGA_MEAS_THETA, 0, theta);
	}
    double z = cvGetReal2D(x_k, BELUGA_STATE_Z, 0);    
    double z_noise = 0;
    if(n_k)
    {
        /* z noise will be the last element */
        z_noise = cvGetReal2D(n_k, n_k->rows - 1, 0);
    }

    z += z_noise;
    if(z < 0)
    {
        z = 0;
    }

    // MAYBE: not strictly necessary
    if(z > BelugaDynamicsParameters::m_dWaterDepth)
    {
        z = BelugaDynamicsParameters::m_dWaterDepth;
    }
    
    cvSetReal2D(z_k, nrows-1, 0, z);
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
    double x     = cvGetReal2D(x_k, BELUGA_STATE_X, 0);
    double y     = cvGetReal2D(x_k, BELUGA_STATE_Y, 0);
    double z     = cvGetReal2D(x_k, BELUGA_STATE_Z, 0);
    double zdot  = cvGetReal2D(x_k, BELUGA_STATE_ZDOT, 0);
    double speed = cvGetReal2D(x_k, BELUGA_STATE_SPEED, 0);
    double theta = cvGetReal2D(x_k, BELUGA_STATE_THETA, 0);
    double omega = cvGetReal2D(x_k, BELUGA_STATE_OMEGA, 0);    

    double x_p     = cvGetReal2D(X_p, BELUGA_STATE_X, 0);
	double y_p	   = cvGetReal2D(X_p, BELUGA_STATE_Y, 0);
	double z_p	   = cvGetReal2D(X_p, BELUGA_STATE_Z, 0);
	double zdot_p  = cvGetReal2D(X_p, BELUGA_STATE_ZDOT, 0);
	double speed_p = cvGetReal2D(X_p, BELUGA_STATE_SPEED, 0);
	double theta_p = cvGetReal2D(X_p, BELUGA_STATE_THETA, 0);
	double omega_p = cvGetReal2D(X_p, BELUGA_STATE_OMEGA, 0);	 


    x = check_nan(x, x_p, 0);
    y = check_nan(y, y_p, 0);
    z = check_nan(z, z_p, water_depth);
    zdot = check_nan(zdot, zdot_p, 0);
    speed = check_nan(speed, speed_p, 0.1);
    theta = check_nan(theta, theta_p, 0);
    omega = check_nan(omega, omega_p, 0);

	if(x*x + y*y > tank_radius)
	{
		double phi = atan2(y, x);
		x = 0.95*tank_radius*cos(phi);
		y = 0.95*tank_radius*sin(phi);
	}

    z = MT_CLAMP(z, 0, water_depth);
    zdot = MT_CLAMP(zdot, -BELUGA_CONSTRAINT_MAX_VERTICAL_SPEED,
                    BELUGA_CONSTRAINT_MAX_VERTICAL_SPEED);
    speed = MT_CLAMP(speed, 0, BELUGA_CONSTRAINT_MAX_SPEED);
    omega = MT_CLAMP(omega, -BELUGA_CONSTRAINT_MAX_TURN_RATE,
                     BELUGA_CONSTRAINT_MAX_TURN_RATE);
    
    cvSetReal2D(x_k, BELUGA_STATE_X, 0, x);		 
	cvSetReal2D(x_k, BELUGA_STATE_Y, 0, y);		 
	cvSetReal2D(x_k, BELUGA_STATE_Z, 0, z);		 
	cvSetReal2D(x_k, BELUGA_STATE_ZDOT, 0, zdot);		 
	cvSetReal2D(x_k, BELUGA_STATE_SPEED, 0, speed);	 
	cvSetReal2D(x_k, BELUGA_STATE_THETA, 0, theta);	 
	cvSetReal2D(x_k, BELUGA_STATE_OMEGA, 0, omega);	 
                
}
