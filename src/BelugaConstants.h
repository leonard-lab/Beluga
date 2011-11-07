#ifndef BELUGA_CONSTANTS
#define BELUGA_CONSTANTS

/**********************************************************************
 * Tracker constants
 **********************************************************************/

/* default parameter values */
const unsigned int DEFAULT_BG_THRESH = 60;
const double DEFAULT_MIN_BLOB_PERIMETER = 10;   
const double DEFAULT_MIN_BLOB_AREA = 10;        
const double DEFAULT_MAX_BLOB_PERIMETER = 1000; 
const double DEFAULT_MAX_BLOB_AREA = 1000;
const unsigned int DEFAULT_SEARCH_AREA_PADDING = 100;

const unsigned int DEFAULT_V_THRESH = 40;
const unsigned int DEFAULT_H_THRESH_HIGH = 130;
const unsigned int DEFAULT_H_THRESH_LOW = 10;
const unsigned int DEFAULT_S_THRESH_LOW = 0;
const unsigned int DEFAULT_S_THRESH_HIGH = 255;

const double DEFAULT_WATER_DEPTH = 2.286;

/* tank radius in m, need not be the actual tank radius,
 * this is just used for constraining the state values */
const double BELUGA_TANK_RADIUS = 10.0;

/**********************************************************************
 * UKF/dynamics constants
 **********************************************************************/

/* state indeces */
enum
{
    BELUGA_STATE_X = 0,
    BELUGA_STATE_Y,
    BELUGA_STATE_Z,
    BELUGA_STATE_ZDOT,    
    BELUGA_STATE_SPEED,
    BELUGA_STATE_THETA,
    BELUGA_STATE_OMEGA,
    
    BELUGA_NUM_STATES /* always the last one! */
};
// TODO: These are fucked up
const unsigned int BELUGA_STATE_ORIENTATION = BELUGA_STATE_THETA;
const unsigned int BELUGA_STATE_HEADING = BELUGA_STATE_THETA;

/* input indeces */
enum
{
    BELUGA_INPUT_VERTICAL_SPEED = 0,
    BELUGA_INPUT_FORWARD_SPEED,
    BELUGA_INPUT_STEERING,
    
    BELUGA_NUM_INPUTS /* always the last one! */
};

/* measurement indeces
 * NOTE:  this assumes a signle (x, y, theta) measurement
 *         (i.e., a single blob in a single camera) -
 *         we handle the n-measurement case in code,
 *         which relies on Z being the last measurement*/
enum
{
    BELUGA_MEAS_X = 0,
    BELUGA_MEAS_Y,
    BELUGA_MEAS_THETA,
    BELUGA_MEAS_Z,

    BELUGA_NUM_MEAS /* always the last one! */
};

/* UKF alpha parameter */
const double BELUGA_UKF_ALPHA = 0.1;

/* fitted dynamics constants */
const double K_t = 1.03;
const double K_d1 = 45.0;
const double m_0 = 7.4; // kg
const double m_1 = 6.0; // kg
const double m_eff = m_0 + m_1; // kg
const double r_1 = 0.35;
const double K_omega = 7.0;
const double eta_up = 5.0e-4;
const double eta_down = 4.0e-4;
const double v_off = 0.2;
const double k_d = 70.0;
const double z_off = 0.75;
const double k_teth = 1.1;
const double k_vp = 11.25; // linear coefficient in voltage->power
                           // curve
const double J = 2.5; // kg*m^2 moment of inertia

/* state constraints */
const double BELUGA_CONSTRAINT_MAX_SPEED = 100.0; 
const double BELUGA_CONSTRAINT_MAX_VERTICAL_SPEED = 100.0;
const double BELUGA_CONSTRAINT_MAX_TURN_RATE = 100.0;


/**********************************************************************
 * Robot constants
 **********************************************************************/

/* Beluga doesn't use any handshaking on the COM port */
const bool BELUGA_HANDSHAKING = false;

const double BELUGA_MAX_SPEED = 45;
const double BELUGA_MAX_VERT_SPEED = 45;
const double BELUGA_MAX_TURN = 68.5;
const double BELUGA_DEFAULT_SPEED_DEADBAND = 0.05;
const double BELUGA_DEFAULT_TURN_DEADBAND = 0.05;

/* reasonable based on one of the robots as of 11/7/11 - DTS*/
const unsigned int BELUGA_DEFAULT_DEPTH_MEAS_AT_SURFACE = 145;
const unsigned int BELUGA_DEFAULT_DEPTH_MEAS_AT_BOTTOM = 600;

const unsigned int BELUGA_SERVO_MIN = 0;
const unsigned int BELUGA_SERVO_MAX = 137;

const unsigned int BELUGA_MAX_COMMAND_LENGTH = 30;

/* the minimum amount of time to allow between
 * successive COM port read/writes */
const double BELUGA_MIN_COMMAND_PERIOD_MSEC = 50.0;

const unsigned char BELUGA_LINEFEED = 10;

/* UP / DOWN buttons (windows only) */
const unsigned int BELUGA_UP_BUTTON = 2;
const unsigned int BELUGA_DOWN_BUTTON = 1;

enum
{
	BELUGA_CONTROL_FWD_SPEED = 0,
	BELUGA_CONTROL_STEERING,
	BELUGA_CONTROL_VERT_SPEED,

	BELUGA_CONTROL_SIZE /* must always be the last one */
};

enum
{
	BELUGA_ROBOT_MEASUREMENT_DEPTH = 0,

	BELUGA_ROBOT_MEASUREMENT_SIZE /* must always be the last one */
};

/**********************************************************************
 * Control constants
 **********************************************************************/

enum
{
    BELUGA_WAYPOINT_X = 0,
    BELUGA_WAYPOINT_Y,
    BELUGA_WAYPOINT_Z,

    BELUGA_WAYPOINT_SIZE /* must always be the last one */
};

/* we can use negative z to specify that we want to maintain depth
 * since z < 0 is beneath the tank */
const double BELUGA_WAYPOINT_SAME_Z = -1.0;

#define BELUGA_MAINTAIN_Z < 0

#endif // BELUGA_CONSTANTS
