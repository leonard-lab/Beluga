#ifndef BELUGAROBOT_H
#define BELUGAROBOT_H

#include "MT_Robot.h"

class Beluga : public MT_RobotBase
{
public:
    Beluga();
    Beluga(const char* onComPort, const char* name);

    virtual ~Beluga();

    void JoyStickControl(std::vector<double> js_axes,
                         unsigned int js_buttons);

    const char* getInfo() const;

	void SendCommand(double fwd_speed, double up_spd, double turn);
    void SendCommand(const char* command);

	void Update(std::vector<double> state);
	double GetX() const;
	double GetY() const;
	double GetTheta() const;
	void SetState(std::vector<double> state);
	void SetControl(std::vector<double> u);
	std::vector<double> GetState();
	std::vector<double> GetControl();
	std::vector<double> GetMeasurements();

	void Control();
	void SafeStop();

    unsigned char IsConnected() const;

	double getDepth();

protected:
    MT_ComIO m_COMPort;
    std::string m_sPort;

	std::vector<double> m_vdState;
	std::vector<double> m_vdControls;
    
    void doCommonInit();

private:
    mutable bool m_bIsConnected;

	unsigned char m_ucDepthByte[6];
	double m_dDepth;

    double m_dMaxSpeed;
    double m_dMaxVertSpeed;
    double m_dMaxTurn;
    double m_dSpeedDeadBand;
    double m_dTurnDeadBand;

    double m_dVertSpeed;

    double m_dMinCommandPeriod_msec;
    
};

#endif // BELUGAROBOT_H
