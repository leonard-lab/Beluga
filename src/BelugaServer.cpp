#include "BelugaServer.h"

#ifdef WITH_SERVER
/**********************************************************************
 * Server Module Class (Optional)
 *********************************************************************/

/* Gets called during server module initialization.  Should define
 * the messages available for this server.  */
MT_Server::t_msg_def* MT_SM_BelugaTracker::getMessageDefs()
{

    /* 2 is the number of messages defined here. Note this is always
       one higher than the actual number of functional messages - the
       last message needs to be MT_Server::sentinel_msg */
    MT_Server::t_msg_def messages[2];

    /* use MT_Server::makeMessage to generate a message object:
       makeMessage(server_code,  <- can be = client_code
                   client_code,  <- unique integer (recommend an enum)
                   description,  <- human-readable description
                   handler)      <- handler object

       server_code will get set automatically by the server. for
       debugging purposes it's recommended to set this to a unique
       integer (e.g. = client_code)

       client_code is how your module will identify this code, so it
       should be a unique integer.  the best way to do this is to use
       an enum.

       the human-readable description is for debugging and open client
       purposes, and should describe what the message corresponds to.

       the handler will generally be "this" and is a pointer to the
       MT_ServerModule that will handle the message, i.e. the server
       will call handler->handleMessage
    */
    messages[msg_GetBlobInfo] = MT_Server::makeMessage(0,
                                                       msg_GetBlobInfo,
                                                       "Get Blob Info",
                                                       this);
    /* the final message needs to be a sentinel.  this flags the end
       of the table to the server. */
    messages[msg_Sentinel] = MT_Server::sentinel_msg;

    /* needs to return the result of setMessages, which takes care of
     * the behind-the-scenes stuff */
    return setMessages(messages);
}

/* this gets called whenever the server encounters a message whose
 * handler is this module.  the msg_code will correspond to the
 * client code established above.  the socket is an object used to
 * actually do the data transfer
 *
 * this function should return true when the message is properly
 * handled and false otherwise (e.g. if the message code is
 * unknown) */
bool MT_SM_BelugaTracker::handleMessage(MT_Server::t_msg msg_code,
                                          wxSocketBase* sock)
{

    /* it's feasible to implement everything in this function, but
       cleaner to thing of this as a switch box.  i.e. it should compare
       the message code to the known codes and then call the
       corresponding response function. */
    if(msg_code == m_pMessages[msg_GetBlobInfo].server_code)
    {
        sendBlobInfo(sock);
        return true;
    }

    return false;
}

/* message handler for when blob info is requested */
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
