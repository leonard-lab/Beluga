#ifndef BELUGAROBOT_H
#define BELUGAROBOT_H

#include "MT_Robot.h"

const double BELUGA_MAX_SPEED = 45;
const double BELUGA_MAX_VERT_SPEED = 45;
const double BELUGA_MAX_TURN = 68.5;
const double BELUGA_DEFAULT_SPEED_DEADBAND = 0.05;
const double BELUGA_DEFAULT_TURN_DEADBAND = 0.05;

const unsigned int BELUGA_SERVO_MIN = 0;
const unsigned int BELUGA_SERVO_MAX = 137;

const unsigned int BELUGA_MAX_COMMAND_LENGTH = 30;

/* UP / DOWN buttons (windows only) */
const unsigned int BELUGA_UP_BUTTON = 2;
const unsigned int BELUGA_DOWN_BUTTON = 1;

enum
{
	BELUGA_STATE_X = 0,
	BELUGA_STATE_Y,
	BELUGA_STATE_HEADING,
	BELUGA_STATE_SPEED,
	BELUGA_STATE_ORIENTATION,

	BELUGA_STATE_SIZE /* must always be the last one */
};

enum
{
	BELUGA_CONTROL_FWD_SPEED = 0,
	BELUGA_CONTROL_STEERING,
	BELUGA_CONTROL_VERT_SPEED,

	BELUGA_CONTROL_SIZE /* must always be the last one */
};

class Beluga : public MT_RobotBase
{
public:
    Beluga();
    Beluga(const char* onComPort, const char* name);

    virtual ~Beluga();

    void JoyStickControl(std::vector<double> js_axes,
                         unsigned int js_buttons);

    const char* getInfo() const;

    void SendCommand(const char* command);

	void Update(std::vector<double> state);
	double GetX() const;
	double GetY() const;
	double GetTheta() const;
	void SetState(std::vector<double> state);
	void SetControl(std::vector<double> u);
	std::vector<double> GetState();
	std::vector<double> GetControl();

	void Control();
	void SafeStop();

    unsigned char IsConnected() const;

protected:
    MT_ComIO m_COMPort;
    std::string m_sPort;

	std::vector<double> m_vdState;
	std::vector<double> m_vdControls;
    
    void doCommonInit();

    void SendVerticalSpeed(double speed);
    void SendSpeed(double speed);
    void SendTurn(double turn);

private:
    mutable bool m_bIsConnected;

    double m_dMaxSpeed;
    double m_dMaxVertSpeed;
    double m_dMaxTurn;
    double m_dSpeedDeadBand;
    double m_dTurnDeadBand;

    double m_dVertSpeed;
    
};

#endif // BELUGAROBOT_H
