
#include "BelugaTrackerGUI.h"

#include "BelugaVideoSetupDialog.h"

const std::string RobotNames[] = {"Beluga 1",
                                  "Beluga 2",
                                  "LifePreserver",
                                  "N/A",
                                  "N/A",
                                  "N/A",
                                  "N/A"};


const int NO_ROBOT = -1;

enum
{
    ID_MENU_POP_ROBOT = MT_RFB_ID_HIGHEST + 10,
    ID_MENU_FILE_CAM_SETUP
};

/**********************************************************************
 * GUI Frame Class
 *********************************************************************/

BelugaTrackerFrame::BelugaTrackerFrame(wxFrame* parent,
                               wxWindowID id,
                               const wxString& title, 
                               const wxPoint& pos, 
                               const wxSize& size,     
                               long style)
  : MT_RobotFrameBase(parent, id, title, pos, size, style),
    m_iNToTrack(1),
	m_dGotoDist(50.0),
	m_dGotoMaxSpeed(15.0),
	m_dGotoTurningGain(25.0),
	m_bControlActive(false),
	m_iGrabbedTrackedObj(NO_ROBOT),
    m_pServer(NULL),
	m_bGotoActive(false),
	m_dGotoX(0),
	m_dGotoY(0)
{

}

void BelugaTrackerFrame::initUserData()
{

    for(unsigned int i = 0; i < 4; i++)
    {
        m_uiaIndexMap[i] = i;
    }
    
    MT_RobotFrameBase::initUserData();
    
    m_CmdLineParser.AddOption(wxT("n"),
                              wxT("num_to_track"),
                              wxT("Number of objects to track. Default is 3."),
                              wxCMD_LINE_VAL_NUMBER);

	m_pPreferences->AddDouble("Goto Cutoff Distance",
                              &m_dGotoDist,
                              MT_DATA_READWRITE,
                              0);
	m_pPreferences->AddDouble("Goto Max Speed",
                              &m_dGotoMaxSpeed,
                              MT_DATA_READWRITE,
                              0);
	m_pPreferences->AddDouble("Goto Turning Gain",
                              &m_dGotoTurningGain,
                              MT_DATA_READWRITE,
                              0);

    std::vector<std::string> botnames;
    for(unsigned int i = 0; i < 7; i++)
    {
        botnames.push_back(RobotNames[i]);
    }
    m_Robots.SetRobotNames(botnames);
    
    setTimer(100);
}

void BelugaTrackerFrame::makeFileMenu(wxMenu* file_menu)
{
    file_menu->Append(ID_MENU_FILE_CAM_SETUP,
                      wxT("Configure Video Sources..."));
    wxFrame::Connect(ID_MENU_FILE_CAM_SETUP,
                     wxEVT_COMMAND_MENU_SELECTED,
                     wxCommandEventHandler(BelugaTrackerFrame::onMenuFileCamSetup));

    file_menu->AppendSeparator();

    MT_RobotFrameBase::makeFileMenu(file_menu);
}

void BelugaTrackerFrame::handleCommandLineArguments(int argc, wxChar** argv)
{

    long temp;
    if(m_CmdLineParser.Found(wxT("n"), &temp))
    {
        m_iNToTrack = temp;
    }

    MT_TrackerFrameBase::handleCommandLineArguments(argc, argv);
}

MT_RobotBase* BelugaTrackerFrame::getNewRobot(const char* config, const char* name)
{
    Beluga* thebot = new Beluga(config, name);
    ReadDataGroupFromXML(m_XMLSettingsFile, thebot->GetParameters());
    return thebot;
}

void BelugaTrackerFrame::doUserControl()
{
	std::vector<double> u;
	u.resize(BELUGA_CONTROL_SIZE, 0.0);

    if(!m_pTracker)
	{
		return;
	}

	if(!m_Robots.IsPhysical(0) || (m_Robots.TrackingIndex[0] == MT_NOT_TRACKED))
	{
		return;
	}

	if(!m_bControlActive)
	{
		return;
	}

	if(!m_bGotoActive || !m_bControlActive)
	{
		m_Robots[0]->SetControl(u);
		m_Robots[0]->Control();
		return;
	}

	if(m_Robots.IsPhysical(2))
	{
		m_dGotoX = m_Robots[2]->GetX();
		m_dGotoY = m_Robots[2]->GetY();
	}

	double dx = m_Robots[0]->GetX() - m_dGotoX;
	double dy = m_Robots[0]->GetY() - m_dGotoY;
	double dth = atan2(dy, -dx) - MT_DEG2RAD*(m_Robots[0]->GetTheta());
	double d = sqrt(dx*dx + dy*dy);
	double spd = 0;
	double vert = 0;
	double turn = 0;

	if(d < m_dGotoDist)
	{
		m_bGotoActive = false;
	}
	else
	{
		if(d > 3.0*m_dGotoDist)
		{
			spd = m_dGotoMaxSpeed;
		}
		else
		{
			spd = m_dGotoMaxSpeed*0.3333*(d/m_dGotoDist);
		}

		turn = m_dGotoTurningGain*sin(dth);
	}

	printf("dx = %f, dy = %f, dth = %f (%f - %f) spd = %f, turn = %f\n",
		dx, dy, MT_RAD2DEG*dth, MT_RAD2DEG*atan2(dy, -dx), m_Robots[0]->GetTheta(), spd, turn);

	u[BELUGA_CONTROL_FWD_SPEED] = -spd;
	u[BELUGA_CONTROL_VERT_SPEED] = vert;
	u[BELUGA_CONTROL_STEERING] = turn;

	m_Robots[0]->SetControl(u);
	m_Robots[0]->Control();

}

void BelugaTrackerFrame::initTracker()
{
    /* TODO: ask for number of objects to track */
    m_pBelugaTracker = new BelugaTracker(m_pCurrentFrame, m_iNToTrack);
    m_pTracker = (MT_TrackerBase *) m_pBelugaTracker;

#ifdef WITH_SERVER    
    m_pServer = new MT_Server;
    // m_pServer->enableDebugOutput();
    m_pServer->doInit();
    m_pServer->registerModule(new MT_SM_BelugaTracker(m_pServer,
                                                        m_pBelugaTracker));
#endif /* WITH_SERVER */    

    /* note - do NOT call MT_TrackerBase::initTracker, which is
     * a placeholder function that sets m_pTracker to NULL! */

	setTimer(100);
}

void BelugaTrackerFrame::updateRobotStatesFromTracker()
{
    int ti;
	std::vector<double> curr_state;
    for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
    {
        ti = m_Robots.TrackingIndex[i];
        if(ti != MT_NOT_TRACKED)
        {
			curr_state = m_pBelugaTracker->getBelugaState(ti);
            m_Robots[i]->Update(curr_state);
        }
    }
}

bool BelugaTrackerFrame::doKeyboardCallback(wxKeyEvent& event)
{
	bool result = MT_DO_BASE_KEY;

	char k = event.GetKeyCode();

	switch(k)
	{
	case 'g':
		m_bControlActive = !m_bControlActive;
		break;
	}

	bool tresult = MT_RobotFrameBase::doKeyboardCallback(event);
	return result && tresult;
}

bool BelugaTrackerFrame::doMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y)
{
	const double click_thresh = 10.0*10.0;
	bool result = MT_DO_BASE_MOUSE;

	if(m_pTracker)
	{
		if(event.RightUp())
		{
			int imin = -1;
			double dmin;
			double d2, dx, dy;
			for(unsigned int i = 0; i < m_iNToTrack; i++)
			{
				dx = viewport_x - m_pBelugaTracker->getBelugaX(i);
				dy = viewport_y - m_pBelugaTracker->getBelugaY(i);
				d2 = dx*dx + dy*dy;
				if(d2 < click_thresh)
				{
					if(imin < 0 || d2 < dmin)
					{
						dmin = d2;
						imin = i;
					}
				}
			}
			m_iGrabbedTrackedObj = imin;
			if(imin != NO_ROBOT)
			{
				wxString label;
				wxMenu pmenu;

				unsigned int np = 0;

				for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
				{
					if(!m_Robots.IsPhysical(i))
					{
						continue;
					}
					np++;
					label.Printf("Identify Robot %s", m_Robots.RobotName[i].c_str());
					pmenu.Append(ID_MENU_POP_ROBOT + i, label);
					Connect(ID_MENU_POP_ROBOT + i, 
						wxEVT_COMMAND_MENU_SELECTED,
						wxCommandEventHandler(BelugaTrackerFrame::onMenuAssign));
				}
				if(np > 0)
				{
					PopupMenu(&pmenu);
					result = MT_SKIP_BASE_MOUSE;
				}
			}
		} /* event.RightUp */

		if(event.LeftUp())
		{
			if(m_bControlActive)
			{
				m_bGotoActive = true;
				m_dGotoX = viewport_x;
				m_dGotoY = getYMax() - viewport_y;
			}
		}
	}

    bool tresult = MT_RobotFrameBase::doMouseCallback(event, viewport_x, viewport_y);
    return tresult && result;
}

void BelugaTrackerFrame::onMenuAssign(wxCommandEvent& event)
{
	if(m_iGrabbedTrackedObj == NO_ROBOT)
	{
		return;
	}
	int robot_selected = event.GetId() - ID_MENU_POP_ROBOT;
	if(robot_selected >= MT_MAX_NROBOTS)
	{
		return;
	}

	m_Robots.TrackingIndex[robot_selected] = m_iGrabbedTrackedObj;
	
	m_iGrabbedTrackedObj = NO_ROBOT;
}

void BelugaTrackerFrame::doUserGLDrawing()
{
	MT_RobotFrameBase::doUserGLDrawing();

	if(m_bGotoActive)
	{
		MT_DrawCircle(m_dGotoX, getYMax() - m_dGotoY, MT_Green, m_dGotoDist);
	}
}

void BelugaTrackerFrame::onMenuFileCamSetup(wxCommandEvent& event)
{

    for(unsigned int i = 0; i < 4; i++)
    {
        printf("Map in: %d -> %d\n", i, m_uiaIndexMap[i]);
    }

    doPause();
    
    std::vector<std::string> camList = m_pCapture->listOfAvailableCameras(4);
    
    Beluga_VideoSetupDialog* dlg = new Beluga_VideoSetupDialog(m_pCapture, camList, m_uiaIndexMap, this);
    dlg->Show();
    dlg->UpdateView();

    int r = dlg->ShowModal();

    for(unsigned int i = 0; i < 4; i++)
    {
        printf("Map out: %d -> %d\n", i, m_uiaIndexMap[i]);
    }

}

/**********************************************************************
 * GUI App Class
 *********************************************************************/

IMPLEMENT_APP(BelugaTrackerApp)
