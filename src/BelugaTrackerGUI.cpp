
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

const unsigned int FRAME_PERIOD_MSEC = 200;

enum
{
    ID_MENU_POP_ROBOT = MT_RFB_ID_HIGHEST + 10,
    ID_MENU_FILE_CAM_SETUP
};

void writeLineToSocket(const char* str, wxSocketBase* sock)
{
	char* c = (char *)calloc(strlen(str)+1, sizeof(char));
	memcpy(c, str, strlen(str));
	c[strlen(str)] = '\n';
	sock->Write(c, (strlen(str)+1)*sizeof(char));
	free(c);
}

std::string readLineFromSocket(wxSocketBase* sock)
{
	sock->SetFlags(wxSOCKET_NOWAIT);
	std::ostringstream ss;

	char c;
	unsigned int i = 0;
	do
	{
		sock->Read(&c, 1);
		if(c >= ' ' && '~')
		{
			ss << c;
		}
	}while(c != '\n' && i < 1024);

	std::string s = ss.str();
	return s;
}

void parseCommandFromSocket(std::string cmd, int* id, double* x, double* y, double* z)
{
	if(cmd.length() < 7)
	{
		return;
	}
	std::istringstream ss(cmd);

	float tmp = 0;
	ss >> tmp;
	*id = tmp;
	ss >> tmp;
	*x = tmp;
	ss >> tmp;
	*y = tmp;
	ss >> tmp;
	*z = tmp;

}

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
	m_iGotoCam(0),
	m_dGotoXC(0),
	m_dGotoYC(0),
	m_dGotoXW(0),
	m_dGotoYW(0),
	m_bCamerasReady(false),
	m_bConnectSocket(false)
{
	for(unsigned int i = 0; i < 4; i++)
	{
		m_pSlaves[i] = NULL;
	}
	m_pBelugaTracker = NULL;
}

BelugaTrackerFrame::~BelugaTrackerFrame()
{
	if(m_pServer) delete m_pServer;
}

void BelugaTrackerFrame::doUserQuit()
{
	m_bCamerasReady = false;
	doPause();
	m_pTimer->Stop();
    this->stopTimedEvents();

	for(unsigned int i = 1; i < 4; i++)
	{
		if(m_pSlaves[i])
		{
			//MT_CameraSlaveFrame* f = dynamic_cast<MT_CameraSlaveFrame*>(m_pSlaves[i]);
			m_pSlaves[i]->setImage(NULL);
			m_pSlaves[i]->prepareToClose();

            m_pSlaves[i]->Destroy();
            m_pSlaves[i] = NULL;
		}
	}

	MT_RobotFrameBase::doUserQuit();
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

	m_pPreferences->AddBool("Connect IPC Server", &m_bConnectSocket);

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

    m_pSetupInfo = new MT_DataGroup("Camera Setup Info");
    m_pSetupInfo->AddString("Quadrant I Calibration",
                            &m_sQuad1CalibrationPath);
    m_pSetupInfo->AddString("Quadrant I Camera",
                            &m_sQuad1Camera);
    m_pSetupInfo->AddString("Quadrant II Calibration",
                            &m_sQuad2CalibrationPath);
    m_pSetupInfo->AddString("Quadrant II Camera",
                            &m_sQuad2Camera);
    m_pSetupInfo->AddString("Quadrant III Calibration",
                            &m_sQuad3CalibrationPath);
    m_pSetupInfo->AddString("Quadrant III Camera",
                            &m_sQuad3Camera);
    m_pSetupInfo->AddString("Quadrant IV Calibration",
                            &m_sQuad4CalibrationPath);
    m_pSetupInfo->AddString("Quadrant IV Camera",
                            &m_sQuad4Camera);
    m_pSetupInfo->AddString("Quadrant I Mask",
                            &m_sQuad1MaskPath);
    m_pSetupInfo->AddString("Quadrant II Mask",
                            &m_sQuad2MaskPath);
    m_pSetupInfo->AddString("Quadrant III Mask",
                            &m_sQuad3MaskPath);
    m_pSetupInfo->AddString("Quadrant IV Mask",
                            &m_sQuad4MaskPath);
                            
    
    setTimer(FRAME_PERIOD_MSEC);
}

void BelugaTrackerFrame::writeUserXML()
{
    MT_RobotFrameBase::writeUserXML();

    WriteDataGroupToXML(&m_XMLSettingsFile, m_pSetupInfo);
}

void BelugaTrackerFrame::readUserXML()
{
    MT_RobotFrameBase::readUserXML();

    ReadDataGroupFromXML(m_XMLSettingsFile, m_pSetupInfo);
}

void BelugaTrackerFrame::makeFileMenu(wxMenu* file_menu)
{
    file_menu->Append(ID_MENU_FILE_CAM_SETUP,
                      wxT("Configure Video Sources..."));
    wxFrame::Connect(ID_MENU_FILE_CAM_SETUP,
                     wxEVT_COMMAND_MENU_SELECTED,
                     wxCommandEventHandler(BelugaTrackerFrame::onMenuFileCamSetup));

    file_menu->Append(MT_TFB_ID_MENU_FILE_SELECT_DATAFILE, wxT("Select Data Output &File..."));
    wxFrame::Connect(MT_TFB_ID_MENU_FILE_SELECT_DATAFILE,
                     wxEVT_COMMAND_MENU_SELECTED,
                     wxCommandEventHandler(MT_TrackerFrameBase::onMenuFileSelectDataFile));

    file_menu->AppendSeparator();

    MT_FrameBase::makeFileMenu(file_menu);
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

	// Fixing robot 0 as tracked object 0 for now
	m_Robots.TrackingIndex[0] = 0;

    return thebot;
}

void BelugaTrackerFrame::doUserStep()
{
	MT_RobotFrameBase::doUserStep();
}

void BelugaTrackerFrame::acquireFrames()
{
	if(m_bCamerasReady)
	{
		for(unsigned int i = 0; i < 4; i++)
		{
			m_pCameraFrames[i] = m_pCapture->getFrame(MT_FC_NEXT_FRAME, m_uiaIndexMap[i]);
			if(i > 0 && m_pSlaves[i])
			{
				m_pSlaves[i]->setCurrentFrame(m_pCameraFrames[i]);
			}
		}
        m_pCurrentFrame = m_pCameraFrames[0];
	}
}

void BelugaTrackerFrame::runTracker()
{
	if(m_pCameraFrames[0])
	{
		std::vector<double> depth, speed, vert, turn, u, z;
		depth.resize(0);
		speed.resize(0);
		vert.resize(0);
		turn.resize(0);

		int ti;
		for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
		{
			ti = m_Robots.TrackingIndex[i];
			if(ti != MT_NOT_TRACKED)
			{
				z = m_Robots[i]->GetMeasurements();
				depth.push_back(z[BELUGA_MEASUREMENT_DEPTH]);

				u = m_Robots[i]->GetControl();
				speed.push_back(u[BELUGA_CONTROL_FWD_SPEED]);
				vert.push_back(u[BELUGA_CONTROL_VERT_SPEED]);
				turn.push_back(u[BELUGA_CONTROL_STEERING]);
			}
		}
		m_pBelugaTracker->setRobotData(depth, speed, vert, turn);

		printf("Tracking start\n");
		m_pBelugaTracker->doTracking(m_pCameraFrames);
		printf("Tracking done\n");

		if(m_bConnectSocket && !m_Socket.IsConnected())
		{
			wxIPV4address addr;
			addr.Hostname(wxT("127.0.0.1"));
			addr.Service(1234);

			m_Socket.Connect(addr, false);
			m_Socket.WaitOnConnect(2);
			if(!m_Socket.IsConnected())
			{
				MT_ShowErrorDialog(this, wxT("Could not connect to IPC server."));
				m_bConnectSocket = false;
			}
			else
			{
				std::string serverMessage = readLineFromSocket(&m_Socket);
				printf("Success connecting to IPC server, server says:\n%s\n", serverMessage.c_str());
			}
		}
		if(!m_bConnectSocket && m_Socket.IsConnected())
		{
			m_Socket.Close();
		}

	}

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
		m_dGotoXW = m_Robots[2]->GetX();
		m_dGotoYW = m_Robots[2]->GetY();
	}

	double dx = m_dGotoXW - m_Robots[0]->GetX();
	double dy = m_dGotoYW - m_Robots[0]->GetY();
	double dth = (m_Robots[0]->GetTheta()) - atan2(dy, dx);
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

		turn = -m_dGotoTurningGain*sin(dth);
	}

/*	printf("dx = %f, dy = %f, dth = %f (%f - %f) spd = %f, turn = %f\n",
		dx, dy, MT_RAD2DEG*dth, MT_RAD2DEG*atan2(dy, dx), MT_RAD2DEG*m_Robots[0]->GetTheta(), spd, turn);*/

	u[BELUGA_CONTROL_FWD_SPEED] = -spd;
	u[BELUGA_CONTROL_VERT_SPEED] = vert;
	u[BELUGA_CONTROL_STEERING] = turn;

	m_Robots[0]->SetControl(u);
	m_Robots[0]->Control();

}

void BelugaTrackerFrame::initTracker()
{

	if(m_pTracker)
	{
		return;
	}

    if(!m_pSlaves[1])
    {
        MT_ShowErrorDialog(this, wxT("Initialize the video sources first."));
        return;
    }
    
    /* TODO: ask for number of objects to track */
    m_pBelugaTracker = new BelugaTracker(m_pCurrentFrame, m_iNToTrack);
    m_pTracker = (MT_TrackerBase *) m_pBelugaTracker;

	m_pBelugaTracker->setMasks(m_sQuad1MaskPath.c_str(),
			m_sQuad2MaskPath.c_str(),
			m_sQuad3MaskPath.c_str(),
			m_sQuad4MaskPath.c_str());
	m_pBelugaTracker->setCalibrations(m_sQuad1CalibrationPath.c_str(),
			m_sQuad2CalibrationPath.c_str(),
			m_sQuad3CalibrationPath.c_str(),
			m_sQuad4CalibrationPath.c_str());

#ifdef WITH_SERVER    
    m_pServer = new MT_Server;
    // m_pServer->enableDebugOutput();
    m_pServer->doInit();
    m_pServer->registerModule(new MT_SM_BelugaTracker(m_pServer,
                                                        m_pBelugaTracker));
#endif /* WITH_SERVER */    

    for(unsigned int i = 1; i < 4; i++)
    {
        m_pSlaves[i]->setTracker(m_pTracker);
        m_pSlaves[i]->setTrackerFrameGroup(m_pBelugaTracker->getAuxFrameGroup(i-1));
    }

    
    /* note - do NOT call MT_TrackerBase::initTracker, which is
     * a placeholder function that sets m_pTracker to NULL! */

	setTimer(FRAME_PERIOD_MSEC);

}

void BelugaTrackerFrame::updateRobotStatesFromTracker()
{

    if(!m_pTracker)
	{
		return;
	}

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

	if(m_Socket.IsConnected())
	{
		std::ostringstream ss;
		ss << "set position";
		for(int i = 0; i < m_iNToTrack; i++)
		{
			ss << " " << i;
			ss << " " << m_pBelugaTracker->getBelugaX(i);
			ss << " " << m_pBelugaTracker->getBelugaY(i);
			ss << " " << m_pBelugaTracker->getBelugaZ(i);
		}
		writeLineToSocket(ss.str().c_str(), &m_Socket);
		std::string response = readLineFromSocket(&m_Socket);
		writeLineToSocket("get command 0", &m_Socket);
		response = readLineFromSocket(&m_Socket);
		int id = -1;
		double x = -100;
		double y = -100;
		double z = -100;
		parseCommandFromSocket(response, &id, &x, &y, &z);

		if(id == 0)
		{
			if(x != m_dGotoXW || y!= m_dGotoYW)
			{
				m_dGotoXW = x;
				m_dGotoYW = y;
				m_pBelugaTracker->getCameraXYFromWorldXYandDepth(&m_iGotoCam, &m_dGotoXC, &m_dGotoYC, m_dGotoXW, m_dGotoYW, 0, false);
			}
		}
	}
}

bool BelugaTrackerFrame::doSlaveKeyboardCallback(wxKeyEvent& event, int slave_index)
{
	bool result = MT_DO_BASE_KEY;

	char k = event.GetKeyCode();

	switch(k)
	{
	case 'g':
		m_bControlActive = !m_bControlActive;
		break;
	case 'q':
		doQuit();
		break;
	}

	return result;
}

bool BelugaTrackerFrame::doKeyboardCallback(wxKeyEvent& event)
{
	bool result = MT_DO_BASE_KEY;

	char k = event.GetKeyCode();

	switch(k)
	{
	case 'g':
		m_bControlActive = !m_bControlActive;
		m_bGotoActive = m_bControlActive;
		break;
	}

	bool tresult = MT_RobotFrameBase::doKeyboardCallback(event);
	return result && tresult;
}

bool BelugaTrackerFrame::doSlaveMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y, int slave_index)
{
	bool result = MT_DO_BASE_MOUSE;

		if(event.LeftUp())
		{
			if(m_bControlActive)
			{
				m_bGotoActive = true;
				m_iGotoCam = slave_index;
				m_dGotoXC = viewport_x;
				m_dGotoYC = viewport_y;
				if(m_pBelugaTracker)
				{
					double z = 0;
					m_pBelugaTracker->getWorldXYZFromImageXYAndDepthInCamera(
						&m_dGotoXW,
						&m_dGotoYW,
						&z,
						m_dGotoXC,
						m_dGotoYC,
						0,
						false,
						slave_index);
				}
			}
		}

	return result;
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
			for(int i = 0; i < m_iNToTrack; i++)
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
				m_iGotoCam = 0;
				m_dGotoXC = viewport_x;
				m_dGotoYC = viewport_y;
				if(m_pBelugaTracker)
				{
					double z = 0;
					m_pBelugaTracker->getWorldXYZFromImageXYAndDepthInCamera(
						&m_dGotoXW,
						&m_dGotoYW,
						&z,
						m_dGotoXC,
						m_dGotoYC,
						0,
						false,
						0);
				}
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

	if(m_bGotoActive && m_iGotoCam == 0)
	{
		MT_DrawCircle(m_dGotoXC, m_dGotoYC, MT_Green, 15.0*m_dGotoDist);
	}
}

void BelugaTrackerFrame::doSlaveGLDrawing(int slave_index)
{
	if(m_bGotoActive && m_iGotoCam == slave_index)
	{
		MT_DrawCircle(m_dGotoXC, m_dGotoYC, MT_Green, 15.0*m_dGotoDist);
	}
}


void BelugaTrackerFrame::onMenuFileCamSetup(wxCommandEvent& event)
{

    doPause();

    std::vector<std::string> camList = m_pCapture->listOfAvailableCameras(4);

	if(camList.size() < 4)
	{
		MT_ShowErrorDialog(this, 
			wxT("Unable to get list of cameras.  "
			"Make sure the right drivers are installed "
			"and that the caemras are available in SmartView."));
		return;
	}

    std::vector<std::string*> calibList;
    calibList.resize(4);
    calibList[0] = &m_sQuad1CalibrationPath;
    calibList[1] = &m_sQuad2CalibrationPath;
    calibList[2] = &m_sQuad3CalibrationPath;
    calibList[3] = &m_sQuad4CalibrationPath;

    std::vector<std::string*> maskList;
    maskList.resize(4);
    maskList[0] = &m_sQuad1MaskPath;
	maskList[1] = &m_sQuad2MaskPath;
	maskList[2] = &m_sQuad3MaskPath;
	maskList[3] = &m_sQuad4MaskPath;

    Beluga_VideoSetupDialog* dlg = new Beluga_VideoSetupDialog(m_pCapture,
                                                               camList,
                                                               calibList,
                                                               maskList,
                                                               m_uiaIndexMap,
                                                               this);
	registerDialogForXML(dlg);
    dlg->Show();
    dlg->UpdateView();

    int r = dlg->ShowModal();

    m_sQuad1Camera = camList[m_uiaIndexMap[0]];
    m_sQuad2Camera = camList[m_uiaIndexMap[1]];
    m_sQuad3Camera = camList[m_uiaIndexMap[2]];
    m_sQuad4Camera = camList[m_uiaIndexMap[3]];

	m_bCamerasReady = true;

    bool firstTime = false;
    if(!m_pSlaves[0])
    {
        firstTime = true;
    }

	for(unsigned int i = 1; i < 4; i++)
	{
        MT_CameraSlaveFrame* frame = m_pSlaves[i];
        if(!m_pSlaves[i])
        {
            frame = new MT_CameraSlaveFrame(NULL);
            m_pSlaves[i] = frame;
            frame->doMasterInitialization();
            frame->Show();
            frame->Raise();
        }

		m_pCameraFrames[i] = m_pCapture->getFrame(MT_FC_NEXT_FRAME, m_uiaIndexMap[i]);
		m_pSlaves[i]->setCurrentFrame(m_pCameraFrames[i]);
		m_pSlaves[i]->setImage(m_pCameraFrames[i]);
        m_pSlaves[i]->setIndex(i);
		m_pSlaves[i]->setMTParent(this);

        /* don't want the user to be able to close these */
		frame->EnableCloseButton(false);
	}

    if(firstTime)
    {
        m_pSlaves[1]->SetTitle(wxT("View in Quadrant II"));
        m_pSlaves[2]->SetTitle(wxT("View in Quadrant III"));
        m_pSlaves[3]->SetTitle(wxT("View in Quadrant IV"));
        registerDialogForXML(m_pSlaves[1]);
        registerDialogForXML(m_pSlaves[2]);
        registerDialogForXML(m_pSlaves[3]);

        m_pSlaves[0] = NULL;
    }
    
	m_pCameraFrames[0] = m_pCapture->getFrame(MT_FC_NEXT_FRAME, m_uiaIndexMap[0]);

    m_pCurrentFrame = m_pCameraFrames[0];
	setImage(m_pCurrentFrame);
	int framewidth_pixels = m_pCapture->getFrameWidth(m_uiaIndexMap[0]);
	int frameheight_pixels = m_pCapture->getFrameHeight(m_uiaIndexMap[0]);
	setSizeByClient(framewidth_pixels, frameheight_pixels);
    setViewport(MT_BlankRectangle, true);
	lockCurrentViewportAsOriginal();

    if(firstTime)
    {
        setTimer(10);
        m_pTrackerControlFrame->enableButtons();
    
        onNewCapture();
    }

	if(m_pBelugaTracker)
	{
		m_pBelugaTracker->setMasks(m_sQuad1MaskPath.c_str(),
			m_sQuad2MaskPath.c_str(),
			m_sQuad3MaskPath.c_str(),
			m_sQuad4MaskPath.c_str());
		m_pBelugaTracker->setCalibrations(m_sQuad1CalibrationPath.c_str(),
			m_sQuad2CalibrationPath.c_str(),
			m_sQuad3CalibrationPath.c_str(),
			m_sQuad4CalibrationPath.c_str());
	}

}

/**********************************************************************
 * GUI App Class
 *********************************************************************/

IMPLEMENT_APP(BelugaTrackerApp)
