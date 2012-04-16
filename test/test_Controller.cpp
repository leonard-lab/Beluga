#include "BelugaTest.h"

#include "BelugaControl.h"

int testBelugaLowLevelControlLaw()
{
    BelugaLowLevelControlLaw control_law;

    /* the output should have BELUGA_CONTROL_SIZE elements */
    mt_dVector_t u_in, u_out, state;
    u_in.resize(3, 0.0);
    state.resize(4, 0.0);

    START_TEST("Checking output vector size");
    u_out = control_law.doControl(state, u_in);
    if(u_out.size() != BELUGA_CONTROL_SIZE)
        RETURN_ERROR_ELSE_OK("Incorrect size " << u_out.size());

    /* the vertical thrust should be zero when u_vert is zero */
    u_in = randomVector(3);
    state = randomVector(4);
    u_in[BELUGA_CONTROL_VERT_SPEED] = 0.0;
    state[BELUGA_STATE_Z] = z_off;
    
    START_TEST("Checking that the vertical thrust is zero if no vertical command is given");
    u_out = control_law.doControl(state, u_in);
    if(u_out[BELUGA_CONTROL_VERT_SPEED] != 0.0)        
        RETURN_ERROR_ELSE_OK("Control was " << u_out[BELUGA_CONTROL_VERT_SPEED]);
    
    return OK;
}

int testBelugaWaypointControlLaw()
{
    BelugaWaypointControlLaw control_law;
    control_law.doActivate(true);

    std::string err_msg;

    /* the output should have BELUGA_CONTROL_SIZE elements */
    mt_dVector_t u_in, u_out, state;
    u_in.resize(BELUGA_WAYPOINT_SIZE, 0.0);
    state.resize(BELUGA_NUM_STATES, 0.0);

    START_TEST("Checking output vector size");
    u_out = control_law.doControl(state, u_in);
    if(u_out.size() != BELUGA_CONTROL_SIZE)
        RETURN_ERROR_ELSE_OK("Incorrect size " << u_out.size());

    START_TEST("Checking that the speed saturates for large distances");
    u_in[BELUGA_WAYPOINT_X] = state[BELUGA_STATE_X];
    u_in[BELUGA_WAYPOINT_Y] = state[BELUGA_STATE_Y] + 1.1*control_law.m_dDist;
    u_out = control_law.doControl(state, u_in);
    
    /* NOTE: use eq_wf to compare floating point numbers - it checks
     * the values rather than the binary representation  */
    if(!eq_wf(u_out[BELUGA_CONTROL_FWD_SPEED], control_law.m_dMaxSpeed))
        RETURN_ERROR_ELSE_OK("Control did not saturate, was " << u_out[BELUGA_CONTROL_FWD_SPEED]);
        
    return OK;
}

int main(int argc, char** argv)
{
    RUN_TEST_SUITE("Low Level Control", testBelugaLowLevelControlLaw());
    RUN_TEST_SUITE("Waypoint Control", testBelugaWaypointControlLaw());
    
    std::cout << std::endl << "\tAll tests pass!" << std::endl;
    return OK;
}
