#include "Controller.h"

mt_Controller::mt_Controller()
    : m_vqpControlLaws(0),
      m_gc_vpManagedControlLaws(0)
{
}

mt_Controller::~mt_Controller()
{
    doFinalGarbageCollection();
}

void mt_Controller::doFinalGarbageCollection()
{
    for(unsigned int i = 0; i < m_gc_vpManagedControlLaws.size(); i++)
    {
        delete m_gc_vpManagedControlLaws[i];
        m_gc_vpManagedControlLaws[i] = NULL;
    }
}

bool mt_Controller::addManagedControlLaw(mt_ControlLaw* c)
{
    unsigned int in_id = c->getControlLawID();
    for(unsigned int i = 0; i < m_gc_vpManagedControlLaws.size(); i++)
    {
        if(m_gc_vpManagedControlLaws[i]->getControlLawID() == in_id)
        {
            fprintf(stderr, "mt_Controller::addManagedControlLaw warning: "
                    "Control law %d is already being managed by this controller.\n",
                    in_id);
            return false;
        }
    }
    m_gc_vpManagedControlLaws.push_back(c);
    return true;
}

void mt_Controller::doControl()
{
    for(unsigned int bot_num = 0; bot_num < getNumBots(); bot_num++)
    {
        mt_dVector_t control_for_bot = calculateControlFor(bot_num);
    }
}

mt_dVector_t mt_Controller::calculateControlFor(unsigned int bot_num)
{
    mt_dVector_t u_to_now(getNumControlInputsFor(bot_num), 0.0);
    mt_dVector_t state(0, 0.0); // TODO: how do we know how big the
                                // state is?
    for(unsigned int law_num = 0; law_num < getNumLawsFor(bot_num); law_num++)
    {
        u_to_now = runControlLaw(bot_num, law_num, u_to_now, state);
    }
    return u_to_now;
}

unsigned int mt_Controller::getNumControlInputsFor(unsigned int bot_num) const
{
    if(bot_num < getNumBots())
    {
        if(getNumLawsFor(bot_num) > 0)
        {
            return m_vqpControlLaws[bot_num][0]->getNumControlInputs();
        }
        else
        {
            return 0;
        }
    }
    else
    {
        fprintf(stderr, "mt_Controller::getNumControlInputsFor error, "
                "requested robot number %d out of bounds "
                "(have %d)\n", bot_num, getNumBots());
        return 0;
    }    
}

unsigned int mt_Controller::getNumLawsFor(unsigned int bot_num) const 
{
    if(bot_num < getNumBots())
    {
        return m_vqpControlLaws[bot_num].size();
    }
    else
    {
        fprintf(stderr, "mt_Controller::getNumLawsFor error, "
                "requested robot number %d out of bounds "
                "(have %d)\n", bot_num, getNumBots());
        return 0;
    }
}

unsigned int mt_Controller::getNumBots() const 
{
    return m_vqpControlLaws.size();
}

mt_dVector_t mt_Controller::runControlLaw(unsigned int bot_num,
                                          unsigned int law_num,
                                          const mt_dVector_t& u_to_now,
                                          const mt_dVector_t& state)
{
    return m_vqpControlLaws[bot_num][law_num]->doControl(u_to_now, state);
}

bool mt_Controller::ensureControlLawProperties(unsigned int bot_num,
                                               const mt_ControlLaw* p_control_law)
{
    if(bot_num >= getNumBots())
    {
        fprintf(stderr, "mt_Controller::ensureControlLawProperties error, "
                "requested robot number %d out of bounds "
                "(have %d)\n", bot_num, getNumBots());
        return false;
    }
    else if((getNumLawsFor(bot_num) != 0) &&
       (p_control_law->getNumControlInputs() != getNumControlInputsFor(bot_num)))
    {
        fprintf(stderr, "mt_Controller::ensureControlLawProperties error, "
                "number of control inputs mismatch (given %d, have %d)\n",
                p_control_law->getNumControlInputs(),
                getNumControlInputsFor(bot_num));
        return false;
    }
    else
    {
        return true;
    }
}
 
bool mt_Controller::appendControlLawToBot(unsigned int bot_num,
                                          mt_ControlLaw* p_control_law,
                                          bool do_garbage_collection)
{
    if(!ensureControlLawProperties(bot_num, p_control_law))
    {
        fprintf(stderr, "\t In appendControlLawToBot\n");
        return false;
    }
    else
    {
        m_vqpControlLaws[bot_num].push_back(p_control_law);
        if(do_garbage_collection)
        {
            addManagedControlLaw(p_control_law);
        }
        return true;
    }
}

bool mt_Controller::prependControlLawToBot(unsigned int bot_num,
                                           mt_ControlLaw* p_control_law,
                                           bool do_garbage_collection)
{
    if(!ensureControlLawProperties(bot_num, p_control_law))
    {
        fprintf(stderr, "\t In prependControlLawToBot\n");
        return false;
    }
    else
    {
        m_vqpControlLaws[bot_num].push_front(p_control_law);
        if(do_garbage_collection)
        {
            addManagedControlLaw(p_control_law);
        }
        return true;
    }
}

bool mt_Controller::insertControlLawForBot(unsigned int bot_num,
                                           unsigned int before_position,
                                           mt_ControlLaw* p_control_law,
                                           bool do_garbage_collection)
{
    if(!ensureControlLawProperties(bot_num, p_control_law))
    {
        fprintf(stderr, "\t In prependControlLawToBot\n");
        return false;
    }
    else if(before_position >= getNumLawsFor(bot_num))
    {
        fprintf(stderr, "mt_Controller::insertControlLawForBot error, "
                "requested to insert control law before position %d, "
                "but only have %d\n", before_position, getNumLawsFor(bot_num));
        return false;
    }   
    else
    {
        std::deque<mt_ControlLaw*>::iterator it;
        it = m_vqpControlLaws[bot_num].begin() + before_position;
        m_vqpControlLaws[bot_num].insert(it, p_control_law);
        if(do_garbage_collection)
        {
            addManagedControlLaw(p_control_law);
        }
        return true;
    }
}

bool mt_Controller::appendSameControlLawToAll(mt_ControlLaw* p_control_law,
                                              bool do_garbage_collection)
{
    for(unsigned int bot_num = 0; bot_num < getNumBots(); bot_num++)
    {
        if(!appendControlLawToBot(bot_num, p_control_law, mt_CONTROLLER_NO_GC))
        {
            fprintf(stderr, "\t In appendControlLawToAll\n");
            return false;
        }
    }
    if(do_garbage_collection)
    {
        addManagedControlLaw(p_control_law);
    }
    return true;
}

bool mt_Controller::prependSameControlLawToAll(mt_ControlLaw* p_control_law,
                                               bool do_garbage_collection)
{
    for(unsigned int bot_num = 0; bot_num < getNumBots(); bot_num++)
    {
        if(!prependControlLawToBot(bot_num, p_control_law, mt_CONTROLLER_NO_GC))
        {
            fprintf(stderr, "\t In prependControlLawToAll\n");
            return false;
        }
    }
    if(do_garbage_collection)
    {
        addManagedControlLaw(p_control_law);
    }
    return true;
}

bool mt_Controller::insertSameControlLawForAll(unsigned int before_position,
                                               mt_ControlLaw* p_control_law,
                                               bool do_garbage_collection)
{
    for(unsigned int bot_num = 0; bot_num < getNumBots(); bot_num++)
    {
        if(!insertControlLawForBot(bot_num, before_position, p_control_law, mt_CONTROLLER_NO_GC))
        {
            fprintf(stderr, "\t In insertControlLawForAll\n");
            return false;
        }
    }
    if(do_garbage_collection)
    {
        addManagedControlLaw(p_control_law);
    }
    return true;    
}

bool mt_Controller::appendFactoryControlLawToAll(mt_ControlLaw_pFactory_With_ID factory,
                                                 bool do_garbage_collection)
{
    for(unsigned int bot_num = 0; bot_num < getNumBots(); bot_num++)
    {
        unsigned int law_num = getNumLawsFor(bot_num);
        mt_ControlLaw* c = factory(bot_num, law_num);        
        if(!appendControlLawToBot(bot_num, c, do_garbage_collection && mt_CONTROLLER_DO_GC))
        {
            fprintf(stderr, "\t In appendControlLawToAll\n");
            return false;
        }
    }
    return true;    
}

bool mt_Controller::prependFactoryControlLawToAll(mt_ControlLaw_pFactory_With_ID factory,
                                                  bool do_garbage_collection)
{
    for(unsigned int bot_num = 0; bot_num < getNumBots(); bot_num++)
    {
        unsigned int law_num = 0;
        mt_ControlLaw* c = factory(bot_num, law_num);        
        if(!prependControlLawToBot(bot_num, c, do_garbage_collection && mt_CONTROLLER_DO_GC))
        {
            fprintf(stderr, "\t In prependControlLawToAll\n");
            return false;
        }
    }
    return true;    
}

bool mt_Controller::insertFactoryControlLawForAll(unsigned int before_position,
                                                  mt_ControlLaw_pFactory_With_ID factory,
                                                  bool do_garbage_collection)
{
    for(unsigned int bot_num = 0; bot_num < getNumBots(); bot_num++)
    {
        unsigned int law_num = before_position;
        mt_ControlLaw* c = factory(bot_num, law_num);        
        if(!insertControlLawForBot(bot_num, before_position,
                                   c, do_garbage_collection && mt_CONTROLLER_DO_GC))
        {
            fprintf(stderr, "\t In prependControlLawToAll\n");
            return false;
        }
    }
    return true;        
}
