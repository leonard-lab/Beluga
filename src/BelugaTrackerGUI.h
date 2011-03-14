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
#include "BelugaServer.h"

/**********************************************************************
 * GUI Frame Class
 *********************************************************************/

class BelugaTrackerFrame : public MT_RobotFrameBase
{
protected:
    BelugaTracker* m_pBelugaTracker;
    MT_Server* m_pServer;

	double m_dGotoDist;
	double m_dGotoMaxSpeed;
	double m_dGotoTurningGain;

    int m_iNToTrack;
	int m_iGrabbedTrackedObj;

    bool m_bControlActive;
	bool m_bGotoActive;
	double m_dGotoX;
	double m_dGotoY;

    unsigned int m_uiaIndexMap[4];

public:
    BelugaTrackerFrame(wxFrame* parent,
                         wxWindowID id = wxID_ANY,
                         const wxString& title = wxT("Tracker View"), 
                         const wxPoint& pos = wxDefaultPosition, 
                         const wxSize& size = wxSize(640,480),     
                         long style = MT_FIXED_SIZE_FRAME);

    virtual ~BelugaTrackerFrame(){if(m_pServer) delete m_pServer;};

    void makeFileMenu(wxMenu* file_menu);

    void initTracker();
    void initUserData();

	void doUserControl();
	void doUserGLDrawing();

    MT_RobotBase* getNewRobot(const char* config, const char* name);

    void handleCommandLineArguments(int argc, wxChar** argv);
	void updateRobotStatesFromTracker();

	bool doKeyboardCallback(wxKeyEvent &event);
	bool doMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y);
    
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
