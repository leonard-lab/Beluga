#include "BelugaControl.h"

std::string BelugaWaypointControlLaw::s_sName("Beluga Waypoint Controller\n");
std::string BelugaLowLevelControlLaw::s_sName("Beluga Low Level Controller\n");

BelugaWaypointControlLaw* belugaWaypointControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num)
{
    return new BelugaWaypointControlLaw();
}

BelugaLowLevelControlLaw* belugaLowLevelControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num)
{
    return new BelugaLowLevelControlLaw();
}


BelugaWaypointControlLaw::BelugaWaypointControlLaw()
    : mt_ControlLaw(3 /* # control inputs */,
                    3 /* # parameters */),
      m_bActive(false),
      m_dDist(0.5),
      m_dMaxSpeed(30.0),
      m_dTurningGain(20.0)
{
}

mt_dVector_t BelugaWaypointControlLaw::doControl(const mt_dVector_t& state,
                                                 const mt_dVector_t& u_in)
{
    if(!m_bActive || state.size() < BELUGA_NUM_STATES || u_in.size() < BELUGA_WAYPOINT_SIZE)
    {
        return u_in;
    }

    mt_dVector_t u(BELUGA_CONTROL_SIZE, 0.0);

    double x = state[BELUGA_STATE_X];
    double y = state[BELUGA_STATE_Y];
    double z = state[BELUGA_STATE_Z];
    double th = state[BELUGA_STATE_THETA];

    double to_x = u_in[BELUGA_WAYPOINT_X];
    double to_y = u_in[BELUGA_WAYPOINT_Y];
    double to_z = u_in[BELUGA_WAYPOINT_Z];

    if(to_z BELUGA_MAINTAIN_Z)
    {
        to_z = z;
    }

    double dx = to_x - x;
    double dy = to_y - y;
    double dz = to_z - z;

    double dth = atan2(dy, dx) - th;
	dth = atan2(sin(dth), sin(dx));

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
        u_turn = m_dTurningGain*sin(dth);
    }

    u[BELUGA_CONTROL_FWD_SPEED] = u_speed;
    u[BELUGA_CONTROL_VERT_SPEED] = u_vert;
    u[BELUGA_CONTROL_STEERING] = u_turn;    
    
	//printf("Control out: %f, %f, %f\n", u[BELUGA_CONTROL_FWD_SPEED], u[BELUGA_CONTROL_VERT_SPEED], u[BELUGA_CONTROL_STEERING]);

    return u;
}

BelugaLowLevelControlLaw::BelugaLowLevelControlLaw()
    : mt_ControlLaw(3 /* # control inputs */,
                    0 /* # parameters */),
      m_bActive(true)
{
    
}

mt_dVector_t BelugaLowLevelControlLaw::doControl(const mt_dVector_t& state,
                                                 const mt_dVector_t& u_in)
{
    if(!m_bActive || state.size() < 4 || u_in.size() < 3)
    {
        return u_in;
    }

    mt_dVector_t u(BELUGA_CONTROL_SIZE, 0.0);
    
    double u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    double u_vert = u_in[BELUGA_CONTROL_VERT_SPEED];
    double u_turn = u_in[BELUGA_CONTROL_STEERING];

    u[BELUGA_CONTROL_FWD_SPEED] = u_speed;
    u[BELUGA_CONTROL_VERT_SPEED] = u_vert;
    u[BELUGA_CONTROL_STEERING] = u_turn;  

	//printf("Control out: %f, %f, %f\n", u[BELUGA_CONTROL_FWD_SPEED], u[BELUGA_CONTROL_VERT_SPEED], u[BELUGA_CONTROL_STEERING]);

	return u;

}