#include "BelugaControl.h"

std::string BelugaWaypointControlLaw::s_sName("Beluga Waypoint Controller\n");

BelugaWaypointControlLaw::BelugaWaypointControlLaw()
    : mt_ControlLaw(2 /* # control inputs */,
                    2 /* # parameters */),
      m_bActive(false),
      m_dDist(0.5),
      m_dMaxSpeed(1.0),
      m_dTurningGain(1.0)
{
}

mt_dVector_t BelugaWaypointControlLaw::doControl(const mt_dVector_t& state,
                                                 const mt_dVector_t& u_in)
{
    if(!m_bActive || state.size() < 4 || u_in.size() < 3)
    {
        return u_in;
    }

    mt_dVector_t u(3, 0.0);

    double x = state[0];
    double y = state[1];
    double z = state[2];
    double th = state[3];

    double to_x = u_in[0];
    double to_y = u_in[1];
    double to_z = u_in[2];

    double dx = x - to_x;
    double dy = y - to_y;
    double dz = z - to_z;

    double dth = th - atan2(dy, dx);
    double d = sqrt(dx*dx + dy*dy);
    double u_speed = 0;
    double u_vert = 0;
    double u_turn = 0;

    if(d < m_dDist)
    {
        m_bActive = false;
    }
    else
    {
        if(d > 3.0*m_dDist)
        {
            u_speed = m_dMaxSpeed;
        }
        else
        {
            u_speed = m_dMaxSpeed*0.333*(d/m_dDist);
        }
        u_turn = -m_dTurningGain*sin(dth);
    }

    u[BELUGA_CONTROL_FWD_SPEED] = u_speed;
    u[BELUGA_CONTROL_VERT_SPEED] = u_vert;
    u[BELUGA_CONTROL_STEERING] = u_turn;    
    
    return u;
}
