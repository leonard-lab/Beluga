#ifndef CONTROL_LAW_H
#define CONTROL_LAW_H

#include <vector>

#include "MT_Robot.h"

// shorthand for a vector of doubles
typedef std::vector<double> mt_dVector_t;

class mt_ControlUIEvent
{
public:
    enum eventType { none, keyboard, mouse };

    mt_ControlUIEvent(eventType type,
                      double screenX,
                      double screenY,
                      double worldX = 0,
                      double worldY = 0,
                      int camera_id = 0);

    bool isKeyboard() const {return m_eventType == keyboard;};
    bool isMouse() const {return m_eventType == mouse;};

    double getScreenX() const {return m_dScreenX;};
    double getScreenY() const {return m_dScreenY;};
    double getWorldX() const {return m_dWorldX;};
    double getWorldY() const {return m_dWorldY;};

    int getCameraID() const {return m_iCameraID;};

    /* factory methods */
    static mt_ControlUIEvent keyboardEvent(double screenX,
                                           double screenY,
                                           double worldX = 0,
                                           double worldY = 0,
                                           int camera_id = 0);
    
    static mt_ControlUIEvent mouseEvent(double screenX,
                                        double screenY,
                                        double worldX = 0,
                                        double worldY = 0,
                                        int camera_id = 0);

private:
    eventType m_eventType;

    double m_dScreenX;
    double m_dScreenY;
    double m_dWorldX;
    double m_dWorldY;

    int m_iCameraID;
};

typedef std::vector<mt_ControlUIEvent> mt_UIEventVector_t;


class mt_ControlLaw
{
public:
    mt_ControlLaw(unsigned int num_control_inputs, unsigned int num_parameters);

    bool setParameters(const mt_dVector_t& new_params);
    mt_dVector_t getParameters() const;

    bool getNumControlInputs() const {return m_iNumControlInputs;};

    mt_dVector_t doControl(const mt_dVector_t& u_to_now,
                           const mt_dVector_t& state)
    {return u_to_now;};

    /* for garbage collection */
    unsigned int getControlLawID() const {return m_iID;};

protected:
    mt_dVector_t m_vParameters;
    
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
