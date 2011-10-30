#ifndef BELUGATRACKERGUI_H
#define BELUGATRACKERGUI_H

#ifdef _WIN32
#include <windows.h>
#endif

/* Include necessary MADTraC headers */
#include "MT_Core.h"
#include "MT_GUI.h"
#include "MT_Tracking.h"
#include "MT_Robot.h"

#include "BelugaRobot.h"
#include "BelugaTracker.h"

/**********************************************************************
 * GUI Frame Class
 *********************************************************************/

class BelugaTrackerFrame : public MT_RobotFrameBase
{
protected:
    BelugaTracker* m_pBelugaTracker;

	double m_dGotoDist;
	double m_dGotoMaxSpeed;
	double m_dGotoTurningGain;

    int m_iNToTrack;
	int m_iGrabbedTrackedObj;

    bool m_bControlActive;
	bool m_bGotoActive;
	int m_iGotoCam;
	double m_dGotoXC;
	double m_dGotoYC;
	double m_dGotoXW;
	double m_dGotoYW;

	bool m_bCamerasReady;
    unsigned int m_uiaIndexMap[4];

	MT_CameraSlaveFrame* m_pSlaves[4];
	IplImage* m_pCameraFrames[4];

    MT_DataGroup* m_pSetupInfo;
    std::string m_sQuad1CalibrationPath;
    std::string m_sQuad1Camera;
    std::string m_sQuad2CalibrationPath;
	std::string m_sQuad2Camera;
    std::string m_sQuad3CalibrationPath;
	std::string m_sQuad3Camera;
    std::string m_sQuad4CalibrationPath;
	std::string m_sQuad4Camera;

    std::string m_sQuad1MaskPath;
    std::string m_sQuad2MaskPath;
    std::string m_sQuad3MaskPath;
    std::string m_sQuad4MaskPath;  

	bool m_bConnectSocket;
	wxSocketClient m_Socket;

	void acquireFrames();
	void runTracker();

    void manageIPCConnection();
    bool tryIPCConnect();

public:
    BelugaTrackerFrame(wxFrame* parent,
                         wxWindowID id = wxID_ANY,
                         const wxString& title = wxT("View in Quadrant I"), 
                         const wxPoint& pos = wxDefaultPosition, 
                         const wxSize& size = wxSize(640,480),     
                         long style = MT_FIXED_SIZE_FRAME);

    virtual ~BelugaTrackerFrame();

	void doUserQuit();

    void makeFileMenu(wxMenu* file_menu);

    void doRobotTimedEvents();

    void initTracker();
    void initUserData();

	void doUserStep();
	void doUserControl();
	void doUserGLDrawing();
	void doSlaveGLDrawing(int slave_index);

    void doIPCExchange();

    void readUserXML();
    void writeUserXML();

    MT_RobotBase* getNewRobot(const char* config, const char* name);

    void handleCommandLineArguments(int argc, wxChar** argv);
	void updateRobotStatesFromTracker();

	bool doKeyboardCallback(wxKeyEvent &event);
	bool doMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y);

	bool doSlaveKeyboardCallback(wxKeyEvent &event, int slave_index);
	bool doSlaveMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y, int slave_index);
    
   /* menu callbacks */
	void onMenuAssign(wxCommandEvent& event);
    void onMenuFileCamSetup(wxCommandEvent& event);

};


/**********************************************************************
 * GUI App Class
 *********************************************************************/

class BelugaTrackerApp
: public MT_AppBase
{
public:
    MT_FrameWithInit* createMainFrame()
    {
        return new BelugaTrackerFrame(NULL);
    };
};


#endif /* BELUGATRACKERGUI_H */
