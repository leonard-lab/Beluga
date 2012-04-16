#ifndef BELUGA_TEST_H
#define BELUGA_TEST_H

#include <vector>
#include <iostream>
#include <sstream>

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

double randomfloat()
{

    double f;
#ifdef _WIN32
    f = ((rand() % 1000) / 1000.0);
#else  
    f = ((rand() % 1000000) / 1000000.0);
#endif
    return 3*(f - 0.5);
}

bool eq_wf(double a, double b)
{
    char a_s[16];
    char b_s[16];

    sprintf(a_s, "%4.3f", a);
    sprintf(b_s, "%4.3f", b);

    return (strncmp(a_s, b_s, 16) == 0);
}

std::vector<double> randomVector(unsigned int size)
{
    std::vector<double> r(size);

    for(unsigned int i = 0; i < size; i++)
    {
        r[i] = randomfloat();
    }
    return r;
}

bool check_doubles_match(double a, double a_exp,
                         double b, double b_exp,
                         double c, double c_exp,
                         const char* a_label,
                         const char* b_label,
                         const char* c_label,
                         std::string* err_msg)
{
    std::ostringstream ss(*err_msg);
    if(!eq_wf(a, a_exp) || !eq_wf(b, b_exp) || !eq_wf(c, c_exp))
    {
        ss << std::endl << "Mismatch error: "
           << "\tExpected " << a_label << "to be " << a_exp << ", got " << a << std::endl
           << "\tExpected " << b_label << "to be " << b_exp << ", got " << b << std::endl
           << "\tExpected " << c_label << "to be " << c_exp << ", got " << c << std::endl;
        *err_msg = ss.str();
        return false;
    }
    return true;
}

bool check_vector_match(const std::vector<double>& a,
                        const std::vector<double>& a_exp,
                        const std::vector<double>& b,
                        const std::vector<double>& b_exp,
                        const std::vector<double>& c,
                        const std::vector<double>& c_exp,
                        const char* a_label,
                        const char* b_label,
                        const char* c_label,
                        std::string* err_msg)
{
    bool r = true;
    for(unsigned int i = 0; i < a.size(); i++)
    {
        r &= check_doubles_match(a[i], a_exp[i], b[i], b_exp[i], c[i], c_exp[i],
                                 a_label, b_label, c_label, err_msg);
    }
    return r;
}

#endif // BELUGA_TEST_H
