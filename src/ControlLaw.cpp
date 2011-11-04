#include "ControlLaw.h"

unsigned int mt_ControlLaw::s_iMaxControlLawID = 0;
std::string mt_ControlLaw::s_sName = std::string("mt_ControlLaw Base");

mt_ControlLaw::mt_ControlLaw(unsigned int num_control_inputs,
                             unsigned int num_parameters)
    : m_iNumControlInputs(num_control_inputs),
      m_vParameters(num_parameters, 0.0),
      m_iID(s_iMaxControlLawID++)
{
}

bool mt_ControlLaw::setParameters(const mt_dVector_t& new_params)
{
    if(new_params.size() == m_vParameters.size())
    {
        m_vParameters = new_params;
        return true;
    }
    else
    {
        fprintf(stderr, "mt_ControlLaw::setParameters size mismatch error. "
                "Input size is %ld, expected %ld\n", new_params.size(),
                m_vParameters.size());
        return false;
    }
}

mt_dVector_t mt_ControlLaw::getParameters() const
{
    return m_vParameters;
}
