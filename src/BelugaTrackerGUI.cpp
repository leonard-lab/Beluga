
#include "BelugaTrackerGUI.h"


/**********************************************************************
 * GUI Frame Class
 *********************************************************************/

BelugaTrackerFrame::BelugaTrackerFrame(wxFrame* parent,
                               wxWindowID id,
                               const wxString& title, 
                               const wxPoint& pos, 
                               const wxSize& size,     
                               long style)
  : MT_TrackerFrameBase(parent, id, title, pos, size, style),
    m_sBackgroundImage(wxEmptyString),
    m_iNToTrack(1),
    m_iStartFrame(-1),
    m_iStopFrame(-1),
    m_iThreshFromCmdLine(-1),
    m_sNoteFromCommandLine(wxEmptyString),
    m_pServer(NULL)
{
    /* nothing else to do here */
}

void BelugaTrackerFrame::initUserData()
{
    initTrackerFrameData();

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
    m_pServer->doInit();
    m_pServer->registerModule(new MT_SM_BelugaTracker(m_pServer,
                                                        m_pBelugaTracker));
#endif /* WITH_SERVER */    

    /* note - do NOT call MT_TrackerBase::initTracker, which is
     * a placeholder function that sets m_pTracker to NULL! */
}

#ifdef WITH_SERVER
/**********************************************************************
 * Server Module Class (Optional)
 *********************************************************************/

MT_Server::t_msg_def* MT_SM_BelugaTracker::getMessageDefs()
{
    MT_Server::t_msg_def messages[2];

    messages[msg_GetBlobInfo] = MT_Server::makeMessage(0,
                                                       msg_GetBlobInfo,
                                                       "Get Blob Info",
                                                       this);
    messages[msg_Sentinel] = MT_Server::sentinel_msg;

    return setMessages(messages);
}

bool MT_SM_BelugaTracker::handleMessage(MT_Server::t_msg msg_code,
                                          wxSocketBase* sock)
{
    if(msg_code == m_pMessages[msg_GetBlobInfo].server_code)
    {
        sendBlobInfo(sock);
        return true;
    }

    return false;
}

void MT_SM_BelugaTracker::sendBlobInfo(wxSocketBase* sock)
{
    /* pull the data group from the tracker */
    MT_DataReport* dr_blob = m_pBelugaTracker->getDataReport(0);
    if(!dr_blob)
    {
        return;
    }
    
    /* number of blobs */
    unsigned int n_blobs = dr_blob->GetVectorLength(0);
    MT_SendInt(n_blobs, sock);

    if(n_blobs == 0)
    {
        return;
    }

    double* c_data = (double *)calloc(n_blobs, sizeof(double));
    memset(c_data, 0, n_blobs*sizeof(double));

    for(unsigned int i = 0; i < n_blobs; i++)
    {
        c_data[i] = dr_blob->GetNumericValue(0, i);
    }

    MT_SendDoubleArray(c_data, n_blobs, sock);

    for(unsigned int i = 0; i < n_blobs; i++)
    {
        c_data[i] = dr_blob->GetNumericValue(1, i);
    }

    MT_SendDoubleArray(c_data, n_blobs, sock);

    free(c_data);
    
}
#endif /* WITH_SERVER */

/**********************************************************************
 * GUI App Class
 *********************************************************************/

IMPLEMENT_APP(BelugaTrackerApp)
