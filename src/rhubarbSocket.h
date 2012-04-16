#ifndef CP_SOCKET
#define CP_SOCKET

/* NOTE: This file was manually copied from 
 * rhubarb 0.1.1 on 4/16/12  - DTS */

/*
 *  rhubarbSocket.h - functions for connecting to rhubarb
 *  servers via c++
 *
 *  Compiling:  Shouldn't require anything other than including
 *  this header.  Requires a c++ compiler.
 *
 *  Should actually be relatively applicapble to any client
 *  connection that transfers line-by-line
 *
 *  Copyright (c) 2011 Daniel T. Swain
 *  See the file license.txt for copying permissions
 *
 */

#ifndef WIN32
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#else // defined WIN32
#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
#endif

#include <string.h>

#include <iostream>
#include <sstream>

typedef int rhubarb_socket_t;

const rhubarb_socket_t RHUBARB_SOCKET_NONE = -1;

inline std::string recvRhubarbLine(rhubarb_socket_t sock, int buff_size = 1024)
{
    std::ostringstream ss;
    char* buffer = new char[buff_size];

    int nread = 0;
    int d = 0;
    char lastchar = 0;
    do
    {
        memset(buffer, 0, sizeof(buffer));

        d = recv(sock, buffer, buff_size, 0);
        if(d > 0)
        {
            nread += d;
			for(unsigned int i = 0; i < d; i++)
			{
			    ss << buffer[i];
			}
            lastchar = buffer[d-1];
        }
        
    } while(nread < buff_size && d >= 0 && lastchar != 10);

	delete[] buffer;

    std::string result = ss.str();
    result = result.substr(0, result.length()-1);
    return result;
    
}

inline int sendRhubarbLine(rhubarb_socket_t sock, const char* line)
{
    std::string _line(line);
    if(line[strlen(line)] != '\n')
    {
        _line += "\n";
    }
    return send(sock, _line.c_str(), _line.length(), 0);
}

inline std::string rhubarbMessage(rhubarb_socket_t sock, const char* message)
{
    sendRhubarbLine(sock, message);
    return recvRhubarbLine(sock);
}

inline rhubarb_socket_t getRhubarbSocket(const char* hostname,
                                  int port,
                                  std::string* message = NULL)
{
    struct sockaddr_in sa;
    struct hostent* server;

#ifdef WIN32
    WSADATA wsaData;
    if(WSAStartup(MAKEWORD(2, 0), &wsaData) != 0)
    {
        std::cerr << "Unable to start Winsock\n";
        return RHUBARB_SOCKET_NONE;
    }
#endif    

    server = gethostbyname(hostname);
    if(!server)
    {
        std::cerr << "Unable to determine hostname\n";
        return RHUBARB_SOCKET_NONE;
    } 

    memset(&sa, 0, sizeof(sa));

    sa.sin_family = AF_INET;
    memcpy((char *) &sa.sin_addr.s_addr,
		   (char *) server->h_addr,          
           server->h_length);
    sa.sin_port = htons(port);

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock < 0)
    {
        std::cerr << "Unable to create socket\n";
        return RHUBARB_SOCKET_NONE;
    }

    if(connect(sock, (struct sockaddr*) &sa, sizeof(sa)) < 0)
    {
        std::cerr << "Unable to connect to server " << hostname
                  << ":" << port << std::endl;
        return RHUBARB_SOCKET_NONE;
    }

    std::string motd = recvRhubarbLine(sock);
    if(message)
    {
        *message = motd;
    }

    return sock;
}

inline void closeRhubarbSocket(rhubarb_socket_t sock)
{
#ifndef WIN32    
    close(sock);
#else
    closesocket(sock);
    WSACleanup();
#endif    
}

#endif // CP_SOCKET
