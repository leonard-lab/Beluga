#ifndef RHUBARB_CLIENT
#define RHUBARB_CLIENT

/* NOTE: This file was manually copied from 
 * rhubarb 0.1.1 on 4/16/12  - DTS */

/*
 *  rhubarbClient.h - Client class for connecting to rhubarb IPC server
 *
 *  Compiling:  Shouldn't require anything other than including
 *  this header (and rhubarbSocket.h).  Requires a c++ compiler.
 *
 *  Should actually be relatively applicapble to any client
 *  connection that transfers line-by-line
 *
 *  Copyright (c) 2011 Daniel T. Swain
 *  See the file license.txt for copying permissions
 *
 */

#include "rhubarbSocket.h"

class rhubarbClient
{
public:
    rhubarbClient(const char* hostname, int port)
        : m_Socket(RHUBARB_SOCKET_NONE),
          m_sHostName(hostname),
          m_iPort(port),
          m_bConnected(false)
    { };
    
    ~rhubarbClient()
    {
        doDisconnect();
    };

    bool doConnect(std::string* motd = NULL)
    {
        std::string welcome_message("");
        m_Socket = getRhubarbSocket(m_sHostName.c_str(), m_iPort, &welcome_message);

        if(m_Socket != RHUBARB_SOCKET_NONE)
        {
            m_bConnected = true;
            if(motd)
            {
                *motd = welcome_message;
            }
        }
        else
        {
            m_bConnected = false;
        }
        return m_bConnected;
    };

    std::string doMessage(const char* message)
    {
        std::string response("ERROR NOT CONNECTED");
        if(m_bConnected)
        {
            response = rhubarbMessage(m_Socket, message);
        }
        return response;
    };

    bool doDisconnect()
    {
        closeRhubarbSocket(m_Socket);
        m_Socket = RHUBARB_SOCKET_NONE;
        m_bConnected = false;
        return m_bConnected;
    };

    bool isConnected() const {return m_bConnected;};

private:
    rhubarb_socket_t m_Socket;

    std::string m_sHostName;
    int m_iPort;

    bool m_bConnected;
    
};

#endif // RHUBARB_CLIENT
