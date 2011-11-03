#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <queue>

#include "ControlLaw.h"

const bool mt_CONTROLLER_DO_GC = true;
const bool mt_CONTROLLER_NO_GC = false;

typedef std::deque<mt_ControlLaw*> mt_pControlLawContainer_t;

class mt_Controller {
public:
    mt_Controller();
    virtual ~mt_Controller();
        
    void doControl();

    mt_dVector_t calculateControlFor(unsigned int bot_num);

    unsigned int getNumControlInputsFor(unsigned int bot_num) const;
    unsigned int getNumLawsFor(unsigned int bot_num) const;
    unsigned int getNumBots() const;

    bool appendControlLawToBot(unsigned int bot_num,
                               mt_ControlLaw* p_control_law,
                               bool do_garbage_collection = mt_CONTROLLER_DO_GC);
    bool prependControlLawToBot(unsigned int bot_num,
                                mt_ControlLaw* p_control_law,
                                bool do_garbage_collection = mt_CONTROLLER_DO_GC);
    bool insertControlLawForBot(unsigned int bot_num,
                                unsigned int before_position,
                                mt_ControlLaw* p_control_law,
                                bool do_garbage_collection = mt_CONTROLLER_DO_GC);

    /* IMPORTANT NOTE:
     * These methods will use the SAME control law object for all
     * robots.  I.e., the pointers all point to the same object
     * in memory.  This will be useful if you want the same functionality
     * with the same parameters all of the time.  You will have to
     * be very careful about concurrency, though!  (Concurrency will
     * be an issue if threads become involved).  If you are not
     * absolutely sure that this is what you want, use the factory
     * versions below.  */
    bool appendSameControlLawToAll(mt_ControlLaw* p_control_law,
                                   bool do_garbage_collection = mt_CONTROLLER_DO_GC);
    bool prependSameControlLawToAll(mt_ControlLaw* p_control_law,
                                    bool do_garbage_collection = mt_CONTROLLER_DO_GC);    
    bool insertSameControlLawForAll(unsigned int before_position,
                                    mt_ControlLaw* p_control_law,
                                    bool do_garbage_collection = mt_CONTROLLER_DO_GC);
    
    /* These functions use a factor method to create make sure that each
     * robot has a SEPARATE COPY of the control law. */
    bool appendFactoryControlLawToAll(mt_ControlLaw_pFactory_With_ID factory,
                                      bool do_garbage_collection = mt_CONTROLLER_DO_GC);
    bool prependFactoryControlLawToAll(mt_ControlLaw_pFactory_With_ID factory,
                                       bool do_garbage_collection = mt_CONTROLLER_DO_GC);
    bool insertFactoryControlLawForAll(unsigned int before_position,
                                       mt_ControlLaw_pFactory_With_ID factory,
                                       bool do_garbage_collection = mt_CONTROLLER_DO_GC);
    
protected:
    mt_dVector_t runControlLaw(unsigned int bot_num,
                               unsigned int law_num,
                               const mt_dVector_t& u_to_now,
                               const mt_dVector_t& state);
    
    std::vector<mt_pControlLawContainer_t> m_vqpControlLaws;

    bool ensureControlLawProperties(unsigned int bot_num, const mt_ControlLaw* p_control_law);

    /**********************************************************************
     * Garbage collection methods
     *  (Don't mess with these unless you're VERY sure of
     *   what you're doing.)
     **********************************************************************/
    std::vector<mt_ControlLaw*> m_gc_vpManagedControlLaws;
    bool addManagedControlLaw(mt_ControlLaw* c);
    void doFinalGarbageCollection();
        
};

#endif // CONTROLLER_H
