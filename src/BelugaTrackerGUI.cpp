
#include "BelugaTrackerGUI.h"

const std::string RobotNames[] = {"Beluga 1",
                                  "Beluga 2",
                                  "N/A",
                                  "N/A",
                                  "N/A",
                                  "N/A",
                                  "N/A"};


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
    m_sBackgroundImage(wxEmptyString),
    m_iNToTrack(1),
    m_iStartFrame(-1),
    m_iStopFrame(-1),
    m_iThreshFromCmdLine(-1),
    m_sNoteFromCommandLine(wxEmptyString),
    m_pServer(NULL)
{

}

void BelugaTrackerFrame::initUserData()
{
    
    MT_RobotFrameBase::initUserData();
    

    m_CmdLineParser.AddOption(wxT("f"),
                              wxT("frames"),
                              wxT("Frame range or start frame. Default is all frames."),
                              wxCMD_LINE_VAL_STRING);

    m_CmdLineParser.AddOption(wxT("n"),
                              wxT("num_to_track"),
                              wxT("Number of objects to track. Default is 3."),
                              wxCMD_LINE_VAL_NUMBER);

    m_CmdLineParser.AddOption(wxT("t"),
                              wxT("threshold"),
                              wxT("Threshold for background subtraction."),
                              wxCMD_LINE_VAL_NUMBER);

    m_CmdLineParser.AddOption(wxT("N"),
                              wxT("Note"),
                              wxT("Note for data file."),
                              wxCMD_LINE_VAL_STRING);

    std::vector<std::string> botnames;
    for(unsigned int i = 0; i < 7; i++)
    {
        botnames.push_back(RobotNames[i]);
    }

    m_Robots.SetRobotNames(botnames);
    
    setTimer(100);
}

void BelugaTrackerFrame::handleCommandLineArguments(int argc, wxChar** argv)
{

    wxString r;
    if(m_CmdLineParser.Found(wxT("f"), &r))
    {
        wxString b = r.BeforeFirst(wxT(':'));
        wxString a = r.AfterFirst(wxT(':'));

        long s1 = 0;
        long s2 = 0;

        bool found_s1 = b.ToLong(&s1);
        bool found_s2 = a.ToLong(&s2);

        if(s1 >= s2)
        {
            s2 = 0; /* will be interpreted as last frame */
        }
        m_iStartFrame = s1;
        m_iStopFrame = s2;

    }

    if(m_CmdLineParser.Found(wxT("N"), &r))
    {
        m_sNoteFromCommandLine = r;
    }
    
    long temp;
    if(m_CmdLineParser.Found(wxT("n"), &temp))
    {
        m_iNToTrack = temp;
    }
    if(m_CmdLineParser.Found(wxT("t"), &temp))
    {
        m_iThreshFromCmdLine = temp;
    }

    MT_TrackerFrameBase::handleCommandLineArguments(argc, argv);
}

MT_RobotBase* BelugaTrackerFrame::getNewRobot(const char* config, const char* name)
{
    Beluga* thebot = new Beluga(config, name);
    ReadDataGroupFromXML(m_XMLSettingsFile, thebot->GetParameters());
    return thebot;
}

void BelugaTrackerFrame::doUserStep()
{
    MT_TrackerFrameBase::doUserStep();

    if(m_pCapture)
    {
        if(m_iStopFrame > 0 && m_pCapture->getFrameNumber() > m_iStopFrame)
        {
            doQuit();
        }
    }
}

void BelugaTrackerFrame::onNewCapture()
{
    if(m_iStartFrame > 2 && m_pCapture)
    {
        int f = m_pCapture->setFrameNumber(m_iStartFrame-2);
        doStep();
        doStep();
    }
}

void BelugaTrackerFrame::initTracker()
{
    /* TODO: ask for number of objects to track */
    m_pBelugaTracker = new BelugaTracker(m_pCurrentFrame, m_iNToTrack);
    m_pTracker = (MT_TrackerBase *) m_pBelugaTracker;
    if(m_iThreshFromCmdLine > 0)
    {
        m_pBelugaTracker->setDiffThresh(m_iThreshFromCmdLine);
    }
    m_pBelugaTracker->setStartStopFrames(m_iStartFrame, m_iStopFrame);
    if(m_sNoteFromCommandLine != wxEmptyString)
    {
        m_pBelugaTracker->setNote(m_sNoteFromCommandLine.mb_str());
    }

#ifdef WITH_SERVER    
    m_pServer = new MT_Server;
    m_pServer->enableDebugOutput();
    m_pServer->doInit();
    m_pServer->registerModule(new MT_SM_BelugaTracker(m_pServer,
                                                        m_pBelugaTracker));
#endif /* WITH_SERVER */    

    /* note - do NOT call MT_TrackerBase::initTracker, which is
     * a placeholder function that sets m_pTracker to NULL! */
}

/**********************************************************************
 * GUI App Class
 *********************************************************************/

IMPLEMENT_APP(BelugaTrackerApp)
