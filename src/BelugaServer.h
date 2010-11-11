#ifndef BELUGA_SERVER_H
#define BELUGA_SERVER_H

/* Include necessary MADTraC headers */
#include "MT_Core.h"
#include "MT_GUI.h"
#include "MT_Tracking.h"

#include "BelugaTracker.h"

#ifdef WITH_SERVER
/**********************************************************************
 * Server Module Class (Optional)
 *********************************************************************/

class MT_SM_BelugaTracker : public MT_ServerModule
{
private:
    BelugaTracker* m_pBelugaTracker;

protected:
    bool handleMessage(MT_Server::t_msg msg_code, wxSocketBase* sock);
    MT_Server::t_msg_def* getMessageDefs();

    enum
    {
        msg_GetBlobInfo = 0,
        msg_Sentinel
    } msg_index;

public:
    MT_SM_BelugaTracker()
        : m_pBelugaTracker(NULL), MT_ServerModule("BelugaTracker"){};
    MT_SM_BelugaTracker(MT_Server* pServer, BelugaTracker* pTracker)
        : m_pBelugaTracker(pTracker),
          MT_ServerModule(pServer, "BelugaTracker"){};

    void sendBlobInfo(wxSocketBase* sock);

    void getBlobInfo(wxSocketBase* sock);

};
#endif /* WITH_SERVER */    

#endif /* BELUGA_SERVER_H */
