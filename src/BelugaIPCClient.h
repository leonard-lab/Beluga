#ifndef BELUGA_IPC_CLIENT_H
#define BELUGA_IPC_CLIENT_H

#include <vector>

#include "rhubarbClient.h"

class belugaIPCClient : public rhubarbClient
{
public:
    belugaIPCClient(const char* hostname = "127.0.0.1",
                  int port = 1234);

    bool getPosition(unsigned int robot, double* x, double* y, double* z);
    bool getPositions(std::vector<unsigned int> robots,
                      std::vector<double>* X,
                      std::vector<double>* Y,
                      std::vector<double>* Z);
    bool getAllPositions(std::vector<double>* X,
                         std::vector<double>* Y,
                         std::vector<double>* Z);
    
};

#endif // BELUGA_IPC_CLIENT_H
