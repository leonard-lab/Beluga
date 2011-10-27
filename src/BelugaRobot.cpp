#include "BelugaRobot.h"

#include "BelugaConstants.h"

/* "Dummy" Constructor -- used for debugging only */
Beluga::Beluga()
    : MT_RobotBase("Anonymous"),
      m_COMPort("stderr", BELUGA_HANDSHAKING),
      m_sPort("stderr"),
      m_bIsConnected(false)
{
    doCommonInit();
}

/* Normal ctor -- give a COM port and a display name, e.g. "Beluga 1"
 * - every robot should have a unique name */
Beluga::Beluga(const char* onComPort, const char* name)
    : MT_RobotBase(onComPort, name),  /* base class init needs com
                                         port and name */
      /* com port initialization with the right handshaking and baud
         rate set up */
      m_COMPort(onComPort, BELUGA_HANDSHAKING, MT_Baud4800),
      m_sPort(onComPort),    /* string name of the com port */
      m_bIsConnected(false)  /* initialize as not connected (will get
                              * set on calling IsConnected */
{
    /* common Beluga initialization */
    doCommonInit();
}

/* Beluga initialization */
Beluga::~Beluga()
{

	SafeStop();

}

void Beluga::SafeStop()
{
	SendCommand(0, 0, 0);
}

void Beluga::doCommonInit()
{
    /* set default parameter values - these will get loaded from XML
     * if available */
    m_dMaxSpeed = BELUGA_MAX_SPEED;
    m_dMaxVertSpeed = BELUGA_MAX_VERT_SPEED;
    m_dMaxTurn = BELUGA_MAX_TURN;
    m_dSpeedDeadBand = BELUGA_DEFAULT_SPEED_DEADBAND;
    m_dTurnDeadBand = BELUGA_DEFAULT_TURN_DEADBAND;
    m_dVertSpeed = 0;

    /* Adding these as an MT_DataGroup enables GUI adjustment and
     * persistence via XML.  The XML file is keyed on the robot name,
     * so the robot name needs to be unique */
	m_vdState.resize(BELUGA_STATE_SIZE, 0.0),
	m_vdControls.resize(BELUGA_CONTROL_SIZE, 0.0),

    m_pParameters = new MT_DataGroup(std::string(m_sName));
    m_pParameters->AddDouble("Max Speed",
                             &m_dMaxSpeed,
                             MT_DATA_READWRITE,
                             0,
                             BELUGA_MAX_SPEED);
    m_pParameters->AddDouble("Vertical Speed",
                             &m_dVertSpeed,
                             MT_DATA_READWRITE,
                             0,
                             BELUGA_MAX_VERT_SPEED);
    m_pParameters->AddDouble("Max Vertical Speed",
                             &m_dMaxVertSpeed,
                             MT_DATA_READWRITE,
                             0,
                             BELUGA_MAX_VERT_SPEED);
    m_pParameters->AddDouble("Max Turn",
                             &m_dMaxTurn,
                             MT_DATA_READWRITE,
                             0,
                             BELUGA_MAX_TURN);
    m_pParameters->AddDouble("Speed Deadband",
                             &m_dSpeedDeadBand,
                             MT_DATA_READWRITE,
                             0,
                             1.0);
    m_pParameters->AddDouble("Turning Deadband",
                             &m_dTurnDeadBand,
                             MT_DATA_READWRITE,
                             0,
                             1.0);

}

/* function to construct a vertical speed command.  speed is a signed
 * double number */
void Beluga::Update(std::vector<double> state)
{
	SetState(state);
}

void Beluga::SetState(std::vector<double> state)
{
	if(state.size() != BELUGA_STATE_SIZE)
	{
		return;
	}

	m_vdState = state;
}

std::vector<double> Beluga::GetState()
{
	return m_vdState;
}

std::vector<double> Beluga::GetMeasurements()
{
	std::vector<double> r;
	r.resize(1);
	r[0] = m_dDepth;
	return r;
}

double Beluga::GetX() const
{
	return m_vdState[BELUGA_STATE_X];
}

double Beluga::GetY() const
{
	return m_vdState[BELUGA_STATE_Y];
}

double Beluga::GetTheta() const
{
	return m_vdState[BELUGA_STATE_ORIENTATION];
}

void Beluga::SetControl(std::vector<double> u)
{
	if(u.size() != BELUGA_CONTROL_SIZE)
	{
		return;
	}

	m_vdControls = u;
}

std::vector<double> Beluga::GetControl()
{
	return m_vdControls;
}

/* function to send a command to a Beluga via the COM port.  Makes
 * sure that the final two bytes of the command are an exclamation
 * mark '!' followed by a line feed (10).  Assumes that the line feed
 * is not already present (TODO: should check for this) */
void Beluga::SendCommand(const char* command)
{
    char cmd[BELUGA_MAX_COMMAND_LENGTH];
    
    /* make sure the command ends with a ! */
    if(*(command + strlen(command)-1) == '!')
    {
        sprintf(cmd, "%s%c", command, BELUGA_LINEFEED);
    }
    else
    {
        sprintf(cmd, "%s!%c", command, BELUGA_LINEFEED);
    }

	int f, v, t;
	f = 0; v = 0; t = 0;
	sscanf(command, "$%03d!%03d!%03d!", &v, &f, &t);

	if(v > 100)
	{
		v = 100 - v;
	}
	if(f > 100)
	{
		f = 100 - f;
	}
	f = -f;
	double half_speed = 0.5*((double) (BELUGA_SERVO_MIN + BELUGA_SERVO_MAX));
	t = t - half_speed;

	m_vdControls[BELUGA_CONTROL_FWD_SPEED] = f;
	m_vdControls[BELUGA_CONTROL_STEERING] = t;
	m_vdControls[BELUGA_CONTROL_VERT_SPEED] = v;

    /* uncomment to see output in console */
	//printf("Sending %s\n", cmd);
    
    m_COMPort.SendCommand(cmd);

	int r;
	r = m_COMPort.ReadData(m_ucDepthByte, 6);

/*	printf("Got %d %d %d %d %d %d %d\n", 
		r,
		m_ucDepthByte[0], 
		m_ucDepthByte[1], 
		m_ucDepthByte[2], 
		m_ucDepthByte[3],
        m_ucDepthByte[4],
		m_ucDepthByte[5]); */
	m_ucDepthByte[4] = 0;
	int d;
	sscanf((const char*) m_ucDepthByte, "%d", &d);
	m_dDepth = d;
	printf("\t -> %f\n", m_dDepth);
}

double Beluga::getDepth()
{
	/* this is wrong, but it will show us the bytes */
	return 100*m_ucDepthByte[0] + m_ucDepthByte[1];
}

/* function to check if the COM port is connected */
unsigned char Beluga::IsConnected() const
{
    m_bIsConnected = (bool) m_COMPort.IsConnected();
    return m_bIsConnected;
}

/* function to return some info about this robot */
const char* Beluga::getInfo() const
{
    std::string info("Beluga Robot:  ");
    info += m_sName;
    info += " on COM port ";
    info += m_sPort;
    return info.c_str();
}

/* function to handle joystick control */
void Beluga::JoyStickControl(std::vector<double> js_axes,
                         unsigned int js_buttons)
{
    double speed = 0;
    double vert = 0;
    double turn = 0;

    double x = -js_axes[0];
    double y = -js_axes[1];
    double w = 0;//js_axes[2];
    double z = 0;//-js_axes[3];

    speed = MT_DeadBandAndScale(y, m_dSpeedDeadBand, m_dMaxSpeed);
    turn = MT_DeadBandAndScale(x, m_dTurnDeadBand, m_dMaxTurn);

#ifndef _WIN32
    vert = MT_DeadBandAndScale(z, m_dSpeedDeadBand, m_dMaxVertSpeed);
#else
	if(js_buttons & BELUGA_UP_BUTTON)
	{
		vert = m_dVertSpeed;
	}
	if(js_buttons & BELUGA_DOWN_BUTTON)
	{
		vert = -m_dVertSpeed;
	}
#endif
	SendCommand(speed, vert, turn);
	m_vdControls[BELUGA_CONTROL_FWD_SPEED] = speed;
	m_vdControls[BELUGA_CONTROL_STEERING] = turn;
	m_vdControls[BELUGA_CONTROL_VERT_SPEED] = vert;
}

void Beluga::SendCommand(double fwd_speed, double up_speed, double turn)
{
	m_vdControls[BELUGA_CONTROL_FWD_SPEED] = fwd_speed;
	m_vdControls[BELUGA_CONTROL_STEERING] = turn;
	m_vdControls[BELUGA_CONTROL_VERT_SPEED] = up_speed;

	unsigned int servo_cmd;
    /* assuming that halfway between the minimum and maximum servo
       positions is the neutral (straight forward) position */
    double half_speed = 0.5*((double) (BELUGA_SERVO_MIN + BELUGA_SERVO_MAX));

    servo_cmd = MT_CLAMP(half_speed + turn,
                         BELUGA_SERVO_MIN,
                         BELUGA_SERVO_MAX);

	unsigned int vert_spd_cmd;
    char vert_d = 0;

    if(up_speed < 0)
    {
        vert_d = 1;
    }
    
    vert_spd_cmd = MT_CLAMP(fabs(up_speed), 0, BELUGA_MAX_VERT_SPEED);
    vert_spd_cmd += 100*vert_d;

    unsigned int spd_cmd;
    char d = 0;

    /* the first digit is 1 if speed is negative */
    if(fwd_speed < 0)
    {
        d = 1;
    }

    spd_cmd = MT_CLAMP(fabs(fwd_speed), 0, BELUGA_MAX_SPEED);
    /* makes sure the first digit is 1 if the speed was negative */
    spd_cmd += 100*d;

	char cmd[] = "$123!456!789!";
	sprintf(cmd, "$%03d!%03d!%03d!", vert_spd_cmd, spd_cmd, servo_cmd);
	SendCommand(cmd);

}

void Beluga::Control()
{
	SendCommand(m_vdControls[BELUGA_CONTROL_FWD_SPEED],
		m_vdControls[BELUGA_CONTROL_VERT_SPEED],
		m_vdControls[BELUGA_CONTROL_STEERING]);
}
