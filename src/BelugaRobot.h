#ifndef BELUGAROBOT_H
#define BELUGAROBOT_H

#include "MT_Robot.h"

#define BYTES_TO_READ 16

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

	std::string GetStatusString();

	double convertDepthMeasurement(int d);

	void Control();
	void SafeStop();

    unsigned char IsConnected() const;

	double getDepth() const;
	unsigned int getDepthMeasurement() const {return m_iDepthMeas;};

	void setWaterDepth(double d){if(d > 0){m_dWaterDepth = d;}};

protected:
    MT_ComIO m_COMPort;
    std::string m_sPort;

	std::vector<double> m_vdState;
	std::vector<double> m_vdControls;
    
    void doCommonInit();

private:
    mutable bool m_bIsConnected;

	unsigned char m_ucDepthByte[BYTES_TO_READ];
	unsigned int m_iDepthMeas;
	double m_dDepth;

    double m_dMaxSpeed;
    double m_dMaxVertSpeed;
    double m_dMaxTurn;
    double m_dSpeedDeadBand;
    double m_dTurnDeadBand;

    double m_dVertSpeed;

    double m_dMinCommandPeriod_msec;

	unsigned int m_iDepthMeasAtSurface;
	unsigned int m_iDepthMeasAtBottom;
	double m_dWaterDepth;

	double m_dTimeOfLastSend;
    
};

/* use this VERY carefully - you should know for sure that
 * the object you're passing in is, in fact, a Beluga */
Beluga* castMTRobotToBeluga(MT_RobotBase* bot);

#endif // BELUGAROBOT_H
