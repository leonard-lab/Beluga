#ifndef BELUGA_IPC_CLIENT_H
#define BELUGA_IPC_CLIENT_H

#include <vector>

#include "rhubarbClient.h"
#include "BelugaConstants.h"

class belugaIPCClient : public rhubarbClient
{
public:
    belugaIPCClient(const char* hostname = "127.0.0.1",
                  int port = 1234);

    bool getPositions(std::vector<unsigned int> robots,
                      std::vector<double>* X,
                      std::vector<double>* Y,
                      std::vector<double>* Z);
    bool getPosition(unsigned int robot, double* x, double* y, double* z);
    bool getAllPositions(std::vector<double>* X,
                         std::vector<double>* Y,
                         std::vector<double>* Z);

    bool setPositions(std::vector<unsigned int> robots,
                      std::vector<double>* X,
                      std::vector<double>* Y,
                      std::vector<double>* Z);
    bool setPosition(unsigned int robot, double* x, double* y, double* z);
    bool setAllPositions(std::vector<double>* X,
                         std::vector<double>* Y,
                         std::vector<double>* Z);

    bool getControls(std::vector<unsigned int> robots,
                     BELUGA_CONTROL_MODE* mode,
                     std::vector<double>* X_or_SPEED,
                     std::vector<double>* Y_or_OMEGA,
                     std::vector<double>* Z_or_ZDOT);

private:
    bool doPositionExchange(std::vector<unsigned int> robots,
                            std::vector<double>* X,
                            std::vector<double>* Y,
                            std::vector<double>* Z,
                            const std::string& op);
    
};

#endif // BELUGA_IPC_CLIENT_H
