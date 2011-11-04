#ifndef CONTROL_LAW_H
#define CONTROL_LAW_H

#include <vector>

#include "MT_Robot.h"

// shorthand for a vector of doubles
typedef std::vector<double> mt_dVector_t;
typedef std::vector<mt_dVector_t> mt_dVectorCollection_t;

class mt_ControlLaw
{
public:
    mt_ControlLaw(unsigned int num_control_inputs, unsigned int num_parameters);

    virtual bool setParameters(const mt_dVector_t& new_params);
    virtual mt_dVector_t getParameters() const;

    virtual bool getNumControlInputs() const {return m_iNumControlInputs;};

    virtual mt_dVector_t doControl(const mt_dVector_t& state,
                                   const mt_dVector_t& u_to_now)
    {return u_to_now;};

    std::string getName() const {return s_sName;};

    /* for garbage collection */
    unsigned int getControlLawID() const {return m_iID;};

protected:
    mt_dVector_t m_vParameters;
    static std::string s_sName;
    
private:
    unsigned int m_iNumControlInputs;

    /* used for garbage collection */
    static unsigned int s_iMaxControlLawID;
    unsigned int m_iID;
};

typedef mt_ControlLaw* (*mt_ControlLaw_pFactory)();
typedef mt_ControlLaw* (*mt_ControlLaw_pFactory_With_ID)(unsigned int bot_num,
                                                         unsigned int law_num);

#endif // CONTROL_LAW_H
