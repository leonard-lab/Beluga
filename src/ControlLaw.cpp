#include "ControlLaw.h"

unsigned int mt_ControlLaw::s_iMaxControlLawID = 0;

mt_ControlUIEvent::mt_ControlUIEvent(eventType type,
                                     double screenX,
                                     double screenY,
                                     double worldX,
                                     double worldY,
                                     int camera_id)
    : m_eventType(type),
      m_dScreenX(screenX),
      m_dScreenY(screenY),
      m_dWorldX(worldX),
      m_dWorldY(worldY),
      m_iCameraID(camera_id)
{
}


mt_ControlUIEvent mt_ControlUIEvent::keyboardEvent(double screenX,
                                                   double screenY,
                                                   double worldX,
                                                   double worldY,
                                                   int camera_id)
{
    return mt_ControlUIEvent(keyboard, screenX, screenY, worldX, worldY, camera_id);
}
    
mt_ControlUIEvent mt_ControlUIEvent::mouseEvent(double screenX,
                                                double screenY,
                                                double worldX,
                                                double worldY,
                                                int camera_id)
{
    return mt_ControlUIEvent(mouse, screenX, screenY, worldX, worldY, camera_id);
}
    
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
