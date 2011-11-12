#include "BelugaIPCClient.h"

const unsigned int MAX_BOTS = 4;

#define ERROR(x) std::cerr << x << std::endl;
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
    ss.str(input->substr(start, start-stop));
    ss >> d;
    result.push_back(d);

    start = find_next_nonspace(s, stop);
    stop = find_next_space(s, start);

    if(start == strlen(s))
    {
        return result;
    }
    ss.str(input->substr(start, start-stop));
    ss >> d;
    result.push_back(d);

    start = find_next_nonspace(s, stop);
    stop = find_next_space(s, start);

    if(start == strlen(s))
    {
        return result;
    }
    ss.str(input->substr(start, start-stop));
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
        ERROR("Unable to extract data from server response: \"" << s_orig << "\"");
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
    return doPositionExchange(robots, X, Y, Z, "get");
}

bool belugaIPCClient::setPositions(std::vector<unsigned int> robots,
                                   std::vector<double>* X,
                                   std::vector<double>* Y,
                                   std::vector<double>* Z)
{
    return doPositionExchange(robots, X, Y, Z, "set");
}

bool belugaIPCClient::doPositionExchange(std::vector<unsigned int> robots,
                                         std::vector<double>* X,
                                         std::vector<double>* Y,
                                         std::vector<double>* Z,
                                         const std::string& op)
{
    if(!X || !Y || !Z)
    {
        ERROR("belugaIPCClient error: one or more inputs are NULL.");
        return false;
    }

    std::ostringstream ss;
    ss << op << " position";
    
    for(unsigned int i = 0; i < robots.size(); i++)
    {
        if(robots[i] >= MAX_BOTS)
        {
            ERROR("belugaIPCClient::getPositions error: robot index "
                  << robots[i] << " out of range.");
            return false;
        }
        else
        {
            ss << " " << robots[i];
        }

        if(!op.compare("set"))
        {
            ss << " " << X->at(i) << " " << Y->at(i) << " " << Z->at(i) << " ";
        }
        
    }

    std::string r = rhubarbClient::doMessage(ss.str().c_str());
    std::string r_orig(r);

    X->resize(0);
    Y->resize(0);
    Z->resize(0);

    for(unsigned int i = 0; i < robots.size(); i++)
    {
        double x, y, z;
        if(!parseThreeDoublesFromString(&r, &x, &y, &z))
        {
            return false;
        }
        X->push_back(x);
        Y->push_back(y);
        Z->push_back(z);
    }

    if(X->size() != robots.size() || Y->size() != robots.size() || Z->size() != robots.size())
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
        ERROR("belugaIPCClient::getPosition error: one or more inputs are NULL.");
        return false;
    }

    if(robot >= MAX_BOTS)
    {
        ERROR("belugaIPCClient::getPosition error: robot index out of range.");
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
        ERROR("belugaIPCClient::setPosition error: one or more inputs are NULL.");
        return false;
    }

    if(robot >= MAX_BOTS)
    {
        ERROR("belugaIPCClient::setPosition error: robot index out of range.");
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
