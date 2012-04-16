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
    bool getControl(unsigned int robot, BELUGA_CONTROL_MODE* mode,
                    double* x, double* y, double* z);
    bool getAllControls(BELUGA_CONTROL_MODE* mode,
                        std::vector<double>* X_or_SPEED,
                        std::vector<double>* Y_or_OMEGA,
                        std::vector<double>* Z_or_ZDOT);

    bool setControls(std::vector<unsigned int> robots,
                     BELUGA_CONTROL_MODE* mode,
                     std::vector<double>* X_or_SPEED,
                     std::vector<double>* Y_or_OMEGA,
                     std::vector<double>* Z_or_ZDOT);
    bool setControl(unsigned int robot, BELUGA_CONTROL_MODE* mode,
                    double* x, double* y, double* z);
    bool setAllControls(BELUGA_CONTROL_MODE* mode,
                        std::vector<double>* X_or_SPEED,
                        std::vector<double>* Y_or_OMEGA,
                        std::vector<double>* Z_or_ZDOT);

    bool getWaypoints(std::vector<unsigned int> robots,
                      std::vector<double>* X,
                      std::vector<double>* Y,
                      std::vector<double>* Z);
    bool getWaypoint(unsigned int robot, double* x, double* y, double* z);
    bool getAllWaypoints(std::vector<double>* X,
                         std::vector<double>* Y,
                         std::vector<double>* Z);

    bool setWaypoints(std::vector<unsigned int> robots,
                      std::vector<double>* X,
                      std::vector<double>* Y,
                      std::vector<double>* Z);
    bool setWaypoint(unsigned int robot, double* x, double* y, double* z);
    bool setAllWaypoints(std::vector<double>* X,
                         std::vector<double>* Y,
                         std::vector<double>* Z);
    
    bool getKinematics(std::vector<unsigned int> robots,
                      std::vector<double>* X,
                      std::vector<double>* Y,
                      std::vector<double>* Z);
    bool getKinematics(unsigned int robot, double* x, double* y, double* z);
    bool getAllKinematics(std::vector<double>* X,
                         std::vector<double>* Y,
                         std::vector<double>* Z);

    bool setKinematics(std::vector<unsigned int> robots,
                      std::vector<double>* X,
                      std::vector<double>* Y,
                      std::vector<double>* Z);
    bool setKinematics(unsigned int robot, double* x, double* y, double* z);
    bool setAllKinematics(std::vector<double>* X,
                         std::vector<double>* Y,
                         std::vector<double>* Z);

    bool getParams(std::string* params);
    bool setParams(std::string* params);

private:
    bool doExchange(std::vector<unsigned int> robots,
                    std::vector<double>* A,
                    std::vector<double>* B,
                    std::vector<double>* C,
                    const std::string& op,
                    BELUGA_CONTROL_MODE* mode = NULL);
    
};

#endif // BELUGA_IPC_CLIENT_H
