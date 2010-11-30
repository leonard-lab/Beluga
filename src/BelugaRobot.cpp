#include "BelugaRobot.h"

const bool BELUGA_HANDSHAKING = false;

Beluga::Beluga()
    : MT_RobotBase("Anonymous"),
      m_COMPort("stderr", BELUGA_HANDSHAKING),
      m_sPort("stderr"),
      m_bIsConnected(false)
{
    doCommonInit();
}
      
Beluga::Beluga(const char* onComPort, const char* name)
    : MT_RobotBase(onComPort, name),
      m_COMPort(onComPort, BELUGA_HANDSHAKING, MT_Baud4800),
      m_sPort(onComPort),
      m_bIsConnected(false)
{
    doCommonInit();
}

Beluga::~Beluga()
{

	SafeStop();

}

void Beluga::SafeStop()
{
	if(IsConnected())
	{
		double t0 = MT_getTimeSec();
		SendSpeed(0);
		while(MT_getTimeSec() - t0 < 0.1){};
		SendVerticalSpeed(0);
		while(MT_getTimeSec() - t0 < 0.2){};
		SendTurn(0);
	}
}

void Beluga::doCommonInit()
{
    m_dMaxSpeed = BELUGA_MAX_SPEED;
    m_dMaxVertSpeed = BELUGA_MAX_VERT_SPEED;
    m_dMaxTurn = BELUGA_MAX_TURN;
    m_dSpeedDeadBand = BELUGA_DEFAULT_SPEED_DEADBAND;
    m_dTurnDeadBand = BELUGA_DEFAULT_TURN_DEADBAND;
    m_dVertSpeed = 0;

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

void Beluga::SendVerticalSpeed(double speed)
{
    char cmd[] = "%123!";
    unsigned int spd_cmd;
    char d = 0;

    if(speed < 0)
    {
        d = 1;
    }
    
    spd_cmd = MT_CLAMP(fabs(speed), 0, BELUGA_MAX_VERT_SPEED);
    spd_cmd += 100*d;
    
    sprintf(cmd, "%%%03d!", spd_cmd);
    SendCommand(cmd);
    
}

void Beluga::SendSpeed(double speed)
{
    char cmd[] = "$123!";
    unsigned int spd_cmd;
    char d = 0;

    if(speed < 0)
    {
        d = 1;
    }
    
    spd_cmd = MT_CLAMP(fabs(speed), 0, BELUGA_MAX_SPEED);
    spd_cmd += 100*d;
    
    sprintf(cmd, "$%03d!", spd_cmd);
    SendCommand(cmd);
    
}

void Beluga::SendTurn(double turn)
{
    char cmd[] = "#123!";
    unsigned int servo_cmd;

    double half_speed = 0.5*((double) (BELUGA_SERVO_MIN + BELUGA_SERVO_MAX));

    servo_cmd = MT_CLAMP(half_speed + turn,
                         BELUGA_SERVO_MIN,
                         BELUGA_SERVO_MAX);
    
    sprintf(cmd, "#%03d!", servo_cmd);
    SendCommand(cmd);
    
}

void Beluga::SendCommand(const char* command)
{
    char cmd[BELUGA_MAX_COMMAND_LENGTH];
    
    /* make sure the command ends with a ! */
    if(*(command + strlen(command)-1) == '!')
    {
        sprintf(cmd, "%s%c", command, 10);
    }
    else
    {
        sprintf(cmd, "%s!%c", command, 10);
    }
    
	printf("Sending %s\n", cmd);
    m_COMPort.SendCommand(cmd);
}


unsigned char Beluga::IsConnected() const
{
    m_bIsConnected = m_COMPort.IsConnected();
    return m_bIsConnected;
}

const char* Beluga::getInfo() const
{
    std::string info("Beluga Robot:  ");
    info += m_sName;
    info += " on COM port ";
    info += m_sPort;
    return info.c_str();
}

void Beluga::JoyStickControl(std::vector<double> js_axes,
                         unsigned int js_buttons)
{
    double speed = 0;
    double vert = 0;
    double turn = 0;

    static unsigned int which_cmd = 0;

    double x = -js_axes[0];
    double y = -js_axes[1];
    double w = js_axes[2];
    double z = js_axes[3];

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

    switch(which_cmd)
    {
    case 0:
        SendSpeed(speed);
        break;
    case 1:
        SendVerticalSpeed(vert);
        break;
    case 2:
        SendTurn(turn);
        break;
    }
    if(++which_cmd == 3)
    {
        which_cmd = 0;
    }
}
