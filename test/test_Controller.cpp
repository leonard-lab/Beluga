#include <iostream>

#include "BelugaControl.h"

const int ERROR = -1;
const int OK = 0;

#define START_TEST(x) std::cout << x << "... ";
#define RETURN_ERROR(x) { std::cerr << x << std::endl; return ERROR; }
#define RETURN_ERROR_ELSE_OK(x) RETURN_ERROR(x) else { std::cout << "OK" << std::endl; }

#define DO_TEST(desc, cond, err_msg) \
    START_TEST(desc); \
    if(cond) RETURN_ERROR_ELSE_OK(err_msg);

#define RUN_TEST_SUITE(desc, tester) \
    std::cout << "Starting tests for " << desc << "..." << std::endl; \
    if(tester) return ERROR;

int testBelugaLowLevelControlLaw()
{
    return OK;
}

int main(int argc, char** argv)
{
    RUN_TEST_SUITE("Low Level Control", testBelugaLowLevelControlLaw());

    std::cout << std::endl << "\tAll tests pass!" << std::endl;
    return OK;
}
