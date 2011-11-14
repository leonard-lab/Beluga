#include "BelugaIPCClient.h"

const unsigned int MAX_BOTS = 4;

#define B_ERROR(x) std::cerr << x << std::endl;
size_t find_next_space(const char* str, size_t start)
{
    size_t c = start;
    for(; c < strlen(str); c++)
    {
        if(str[c] == ' ')
        {
            break;
        }
    }
    return c;
}

size_t find_next_nonspace(const char* str, size_t start)
{
    size_t c = start;
    for(; c < strlen(str); c++)
    {
        if(str[c] != ' ')
        {
            break;
        }
    }
    return c;
}

std::string chompWordFromString(std::string* input)
{
    size_t start = find_next_nonspace(input->c_str(), 0);
    size_t stop = find_next_space(input->c_str(), start);

    std::string result = input->substr(start, stop-start);
    stop = find_next_nonspace(input->c_str(), stop);
    input->erase(0, stop);
    return result;
}

std::vector<double> chompThreeDoublesFromString(std::string* input)
{
    std::vector<double> result(0);

    size_t start, stop;
    unsigned int count = 0;
    std::istringstream ss;
    double d;

    const char* s = input->c_str();
    
    start = find_next_nonspace(s, 0);
    stop = find_next_space(s, start);
    
    if(start == strlen(s))
    {
        return result;
    }
    ss.str(input->substr(start, stop-start + 1));
    ss >> d;
    result.push_back(d);

    start = find_next_nonspace(s, stop);
    stop = find_next_space(s, start);

    if(start == strlen(s))
    {
        return result;
    }
    ss.str(input->substr(start, stop-start + 1));
    ss >> d;
    result.push_back(d);

    start = find_next_nonspace(s, stop);
    stop = find_next_space(s, start);

    if(start == strlen(s))
    {
        return result;
    }
    ss.str(input->substr(start, stop-start + 1));
    ss >> d;
    result.push_back(d);

    start = find_next_nonspace(s, stop);
    input->erase(0, start-1);

    return result;
}

bool parseThreeDoublesFromString(std::string* s, double* a, double* b, double* c)
{
    std::string s_orig(*s);
    std::vector<double> d = chompThreeDoublesFromString(s);
    if(d.size() != 3)
    {
        B_ERROR("Unable to extract data from server response: \"" << s_orig << "\"");
        std::cerr << "Got: ";
        for(unsigned int i = 0; i < d.size(); i++)
        {
            std::cerr << d[i] << " ";
        }
        std::cerr << std::endl;
        return false;
    }
    *a = d[0];
    *b = d[1];
    *c = d[2];
    return true;
}

belugaIPCClient::belugaIPCClient(const char* hostname,
                                 int port)
    : rhubarbClient(hostname, port)
{
}

bool belugaIPCClient::getPositions(std::vector<unsigned int> robots,
                                   std::vector<double>* X,
                                   std::vector<double>* Y,
                                   std::vector<double>* Z)
{
    return doExchange(robots, X, Y, Z, "get position");
}

bool belugaIPCClient::setPositions(std::vector<unsigned int> robots,
                                   std::vector<double>* X,
                                   std::vector<double>* Y,
                                   std::vector<double>* Z)
{
    return doExchange(robots, X, Y, Z, "set position");
}

bool belugaIPCClient::getControls(std::vector<unsigned int> robots,
                                  BELUGA_CONTROL_MODE* mode,
                                  std::vector<double>* X_or_SPEED,
                                  std::vector<double>* Y_or_OMEGA,
                                  std::vector<double>* Z_or_ZDOT)
{
    if(!mode)
    {
        std::cerr << "belugaIPCClient::getControls B_ERROR - mode is NULL" << std::endl;
        return false;
    }
    return doExchange(robots, X_or_SPEED, Y_or_OMEGA, Z_or_ZDOT, "get control", mode);
}

bool belugaIPCClient::setControls(std::vector<unsigned int> robots,
                                  BELUGA_CONTROL_MODE* mode,
                                  std::vector<double>* X_or_SPEED,
                                  std::vector<double>* Y_or_OMEGA,
                                  std::vector<double>* Z_or_ZDOT)
{
    if(!mode)
    {
        std::cerr << "belugaIPCClient::setControls B_ERROR - mode is NULL" << std::endl;
        return false;
    }
    return doExchange(robots, X_or_SPEED, Y_or_OMEGA, Z_or_ZDOT, "set control", mode);
}

bool belugaIPCClient::doExchange(std::vector<unsigned int> robots,
                                 std::vector<double>* A,
                                 std::vector<double>* B,
                                 std::vector<double>* C,
                                 const std::string& op,
                                 BELUGA_CONTROL_MODE* mode)
{
    if(!A || !B || !C)
    {
        B_ERROR("belugaIPCClient B_ERROR: one or more inputs are NULL.");
        return false;
    }

    std::ostringstream ss;
    ss << op;

    if(mode && (op.compare(0, 3, "set") == 0))
    {
        switch(*mode)
        {
        case WAYPOINT:
            ss << " waypoint";
            break;
        case KINEMATICS:
            ss << " kinematics";
            break;
        default:
            B_ERROR("belugaIPCClient B_ERROR: Unknown control mode requested.");
            break;
        }
    }
    
    for(unsigned int i = 0; i < robots.size(); i++)
    {
        if(robots[i] >= MAX_BOTS)
        {
            B_ERROR("belugaIPCClient::getPositions B_ERROR: robot index "
                  << robots[i] << " out of range.");
            return false;
        }
        else
        {
            ss << " " << robots[i];
        }

        if(!op.compare(0, 3, "set"))
        {
            ss << " " << A->at(i) << " " << B->at(i) << " " << C->at(i) << " ";
        }
        
    }

    std::string r = rhubarbClient::doMessage(ss.str().c_str());
    std::string r_orig(r);

    if(mode)
    {
        std::string mode_string = chompWordFromString(&r);
        if(!mode_string.compare("waypoint"))
        {
            *mode = WAYPOINT;
        }
        else if(!mode_string.compare("kinematics"))
        {
            *mode = KINEMATICS;
        }
        else
        {
            std::cerr << "belugaIPCClient B_ERROR: Unknown control mode \""
                      << mode_string << "\"" << std::endl;
        }
    }

    A->resize(0);
    B->resize(0);
    C->resize(0);

    for(unsigned int i = 0; i < robots.size(); i++)
    {
        double a, b, c;
        if(!parseThreeDoublesFromString(&r, &a, &b, &c))
        {
            return false;
        }
        A->push_back(a);
        B->push_back(b);
        C->push_back(c);
    }

    if(A->size() != robots.size() || B->size() != robots.size() || C->size() != robots.size())
    {
        std::cerr << "Result size mismatch in belugaIPCClient." << std::endl;
        return false;
    }

    return true;
}

bool belugaIPCClient::getPosition(unsigned int robot, double* x, double* y, double* z)
{
    if(!x || !y || !z)
    {
        B_ERROR("belugaIPCClient::getPosition B_ERROR: one or more inputs are NULL.");
        return false;
    }

    if(robot >= MAX_BOTS)
    {
        B_ERROR("belugaIPCClient::getPosition B_ERROR: robot index out of range.");
        return false;
    }

    std::vector<double> X(1), Y(1), Z(1);
    std::vector<unsigned int> bot(1);

    bot[0] = robot;

    if(!getPositions(bot, &X, &Y, &Z))
    {
        return false;
    }

    *x = X[0];
    *y = Y[0];
    *z = Z[0];
    
    return true;
}

bool belugaIPCClient::setPosition(unsigned int robot, double* x, double* y, double* z)
{
    if(!x || !y || !z)
    {
        B_ERROR("belugaIPCClient::setPosition B_ERROR: one or more inputs are NULL.");
        return false;
    }

    if(robot >= MAX_BOTS)
    {
        B_ERROR("belugaIPCClient::setPosition B_ERROR: robot index out of range.");
        return false;
    }

    std::vector<double> X(1), Y(1), Z(1);
    std::vector<unsigned int> bot(1);

    X[0] = *x;
    Y[0] = *y;
    Z[0] = *z;

    bot[0] = robot;

    if(!setPositions(bot, &X, &Y, &Z))
    {
        return false;
    }

    *x = X[0];
    *y = Y[0];
    *z = Z[0];
    
    return true;
}

bool belugaIPCClient::getControl(unsigned int robot, BELUGA_CONTROL_MODE* mode,
                                 double* x, double* y, double* z)
{
    if(!x || !y || !z || !mode)
    {
        B_ERROR("belugaIPCClient::getControl B_ERROR: one or more inputs are NULL.");
        return false;
    }

    if(robot >= MAX_BOTS)
    {
        B_ERROR("belugaIPCClient::getControl B_ERROR: robot index out of range.");
        return false;
    }

    std::vector<double> X(1), Y(1), Z(1);
    std::vector<unsigned int> bot(1);

    bot[0] = robot;

    if(!getControls(bot, mode, &X, &Y, &Z))
    {
        return false;
    }

    *x = X[0];
    *y = Y[0];
    *z = Z[0];
    
    return true;
}

bool belugaIPCClient::setControl(unsigned int robot, BELUGA_CONTROL_MODE* mode,
                                 double* x, double* y, double* z)
{
    if(!x || !y || !z || !mode)
    {
        B_ERROR("belugaIPCClient::setControl B_ERROR: one or more inputs are NULL.");
        return false;
    }

    if(robot >= MAX_BOTS)
    {
        B_ERROR("belugaIPCClient::setControl B_ERROR: robot index out of range.");
        return false;
    }

    std::vector<double> X(1), Y(1), Z(1);
    std::vector<unsigned int> bot(1);

    bot[0] = robot;

    if(!setControls(bot, mode, &X, &Y, &Z))
    {
        return false;
    }

    *x = X[0];
    *y = Y[0];
    *z = Z[0];
    
    return true;
}

bool belugaIPCClient::getAllPositions(std::vector<double>* X,
                                      std::vector<double>* Y,
                                      std::vector<double>* Z)
{
    std::vector<unsigned int> robots(4);
    robots[0] = 0;
    robots[1] = 1;
    robots[2] = 2;
    robots[3] = 3;
    return getPositions(robots, X, Y, Z);
}

bool belugaIPCClient::setAllPositions(std::vector<double>* X,
                                      std::vector<double>* Y,
                                      std::vector<double>* Z)
{
    std::vector<unsigned int> robots(4);
    robots[0] = 0;
    robots[1] = 1;
    robots[2] = 2;
    robots[3] = 3;
    return setPositions(robots, X, Y, Z);
}

bool belugaIPCClient::getAllControls(BELUGA_CONTROL_MODE* mode,
                                     std::vector<double>* X,
                                     std::vector<double>* Y,
                                     std::vector<double>* Z)
{
    if(!mode)
    {
        std::cerr << "belugaIPCClient::getAllControls B_ERROR: mode input is NULL" << std::endl;
    }
    std::vector<unsigned int> robots(4);
    robots[0] = 0;
    robots[1] = 1;
    robots[2] = 2;
    robots[3] = 3;
    return getControls(robots, mode, X, Y, Z);
}

bool belugaIPCClient::setAllControls(BELUGA_CONTROL_MODE* mode,
                                     std::vector<double>* X,
                                     std::vector<double>* Y,
                                     std::vector<double>* Z)
{
    if(!mode)
    {
        std::cerr << "belugaIPCClient::setAllControls B_ERROR: mode input is NULL" << std::endl;
    }
    std::vector<unsigned int> robots(4);
    robots[0] = 0;
    robots[1] = 1;
    robots[2] = 2;
    robots[3] = 3;
    return setControls(robots, mode, X, Y, Z);
}


bool belugaIPCClient::getWaypoints(std::vector<unsigned int> robots,
                                   std::vector<double>* X,
                                   std::vector<double>* Y,
                                   std::vector<double>* Z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = getControls(robots, &mode, X, Y, Z);
    if(mode != WAYPOINT)
    {
        return false;
    }
    else
    {
        return r;
    }
}

bool belugaIPCClient::getWaypoint(unsigned int robot, double* x, double* y, double* z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = getControl(robot, &mode, x, y, z);
    if(mode != WAYPOINT)
    {
        return false;
    }
    else
    {
        return r;
    }    
}

bool belugaIPCClient::getAllWaypoints(std::vector<double>* X,
                                      std::vector<double>* Y,
                                      std::vector<double>* Z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = getAllControls(&mode, X, Y, Z);
    if(mode != WAYPOINT)
    {
        return false;
    }
    else
    {
        return r;
    }    
}

bool belugaIPCClient::setWaypoints(std::vector<unsigned int> robots,
                                   std::vector<double>* X,
                                   std::vector<double>* Y,
                                   std::vector<double>* Z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = setControls(robots, &mode, X, Y, Z);
    if(mode != WAYPOINT)
    {
        return false;
    }
    else
    {
        return r;
    }    
}

bool belugaIPCClient::setWaypoint(unsigned int robot, double* x, double* y, double* z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = setControl(robot, &mode, x, y, z);
    if(mode != WAYPOINT)
    {
        return false;
    }
    else
    {
        return r;
    }        
}

bool belugaIPCClient::setAllWaypoints(std::vector<double>* X,
                                      std::vector<double>* Y,
                                      std::vector<double>* Z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = setAllControls(&mode, X, Y, Z);
    if(mode != WAYPOINT)
    {
        return false;
    }
    else
    {
        return r;
    }        
}

bool belugaIPCClient::getKinematics(std::vector<unsigned int> robots,
                                   std::vector<double>* X,
                                   std::vector<double>* Y,
                                   std::vector<double>* Z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = getControls(robots, &mode, X, Y, Z);
    if(mode != KINEMATICS)
    {
        return false;
    }
    else
    {
        return r;
    }
}

bool belugaIPCClient::getKinematics(unsigned int robot, double* x, double* y, double* z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = getControl(robot, &mode, x, y, z);
    if(mode != KINEMATICS)
    {
        return false;
    }
    else
    {
        return r;
    }    
}

bool belugaIPCClient::getAllKinematics(std::vector<double>* X,
                                      std::vector<double>* Y,
                                      std::vector<double>* Z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = getAllControls(&mode, X, Y, Z);
    if(mode != KINEMATICS)
    {
        return false;
    }
    else
    {
        return r;
    }    
}

bool belugaIPCClient::setKinematics(std::vector<unsigned int> robots,
                                   std::vector<double>* X,
                                   std::vector<double>* Y,
                                   std::vector<double>* Z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = setControls(robots, &mode, X, Y, Z);
    if(mode != KINEMATICS)
    {
        return false;
    }
    else
    {
        return r;
    }    
}

bool belugaIPCClient::setKinematics(unsigned int robot, double* x, double* y, double* z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = setControl(robot, &mode, x, y, z);
    if(mode != KINEMATICS)
    {
        return false;
    }
    else
    {
        return r;
    }        
}

bool belugaIPCClient::setAllKinematics(std::vector<double>* X,
                                      std::vector<double>* Y,
                                      std::vector<double>* Z)
{
    BELUGA_CONTROL_MODE mode;
    bool r = setAllControls(&mode, X, Y, Z);
    if(mode != KINEMATICS)
    {
        return false;
    }
    else
    {
        return r;
    }        
}
