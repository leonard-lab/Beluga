#ifndef CONTROL_LAW_H
#define CONTROL_LAW_H

#include <vector>

#include "MT_Robot.h"

// shorthand for a vector of doubles
typedef std::vector<double> mt_dVector_t;
typedef std::vector<mt_dVector_t> mt_dVectorCollection_t;

class mt_ControlUIEvent
{
public:
    enum eventType { none = 0, keyboard, mouse };
    enum mouseEventInfo { none = 0,
                          left_button,
                          middle_button,
                          right_button };
                          

    mt_ControlUIEvent(eventType type,
                      double screenX,
                      double screenY,
                      double worldX = 0,
                      double worldY = 0,
                      int camera_id = 0,
                      int event_info = -1);

    bool isKeyboard() const {return m_eventType == keyboard;};
    bool isMouse() const {return m_eventType == mouse;};

    bool isLeftMouse() const {return (m_eventType == mouse && m_iEventInfo == left_button);};
    bool isMiddleMouse() const {return (m_eventType == mouse && m_iEventInfo == middle_button);};
    bool isRightMouse() const {return (m_eventType == mouse && m_iEventInfo == right_button);};

    double getScreenX() const {return m_dScreenX;};
    double getScreenY() const {return m_dScreenY;};
    double getWorldX() const {return m_dWorldX;};
    double getWorldY() const {return m_dWorldY;};

    int getCameraID() const {return m_iCameraID;};

    /* factory methods */
    static mt_ControlUIEvent keyboardEvent(char key,
                                           double screenX,
                                           double screenY,
                                           double worldX = 0,
                                           double worldY = 0,
                                           int camera_id = 0);
    
    static mt_ControlUIEvent mouseEvent(mouseEventInfo button,
                                        double screenX,
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

    unsigned int m_iEventInfo;

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

    mt_dVector_t doControl(const mt_dVector_t& state,
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
