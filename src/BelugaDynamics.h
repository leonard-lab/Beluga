#ifndef BELUGADYNAMICS_H
#define BELUGADYNAMICS_H

#include <cv.h>

class BelugaDynamicsParameters
{
public:
    static double m_dDt;
    static double m_dWaterDepth;
};

void beluga_dynamics(const CvMat* x_k,
                          const CvMat* u_k,
                          const CvMat* v_k,
                          CvMat* x_kplus1);

void beluga_measurement(const CvMat* x_k,
                             const CvMat* n_k,
                             CvMat* z_k);

void constrain_state(CvMat* x_k,
                            CvMat* X_p,
							double tank_radius,
							double water_depth);

#endif // BELUGADYNAMICS_H
