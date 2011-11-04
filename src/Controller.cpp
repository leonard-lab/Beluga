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

bool mt_Controller::validateBotAndLawNum(unsigned int bot_num, unsigned int law_num) const
{
    if(bot_num >= getNumBots())
    {
        fprintf(stderr, "mt_Controller::validateBotAndLawNum error.  Requested "
                "controller for bot %d. Only have %d\n",
                bot_num, getNumBots());
        return false;
    }
    if(law_num > getNumLawsFor(bot_num))
    {
        fprintf(stderr, "mt_Controller::validateBotAndLawNum error.  Requested "
                "law %d for bot %d, but bot %d only has %d laws.\n",
                law_num, bot_num, bot_num, getNumLawsFor(bot_num));
        return false;
    }
    return true;
}

mt_ControlLaw* const mt_Controller::getControlLaw(unsigned int bot_num, unsigned int law_num) const
{
    if(!validateBotAndLawNum(bot_num, law_num))
    {
        fprintf(stderr, "\t In mt_Controller::getControlLaw\n");
        return NULL;
    }
    return m_vqpControlLaws[bot_num][law_num];
}

mt_dVector_t mt_Controller::getParameters(unsigned int bot_num, unsigned int law_num) const
{
    if(!validateBotAndLawNum(bot_num, law_num))
    {
        fprintf(stderr, "\t In mt_Controller::getParameters\n");
        return mt_CONTROLLER_EMPTY_VECTOR;
    }
    return m_vqpControlLaws[bot_num][law_num]->getParameters();
}

bool mt_Controller::setParameters(unsigned int bot_num,
                                  unsigned int law_num,
                                  const mt_dVector_t& parameters)
{
    if(!validateBotAndLawNum(bot_num, law_num))
    {
        fprintf(stderr, "\t In mt_Controller::setParameters\n");
        return false;
    }
    return m_vqpControlLaws[bot_num][law_num]->setParameters(parameters);
}

std::string mt_Controller::getName(unsigned int bot_num, unsigned int law_num) const
{
    if(!validateBotAndLawNum(bot_num, law_num))
    {
        fprintf(stderr, "\t In mt_Controller::getName\n");
        return std::string("ERROR");
    }
    return m_vqpControlLaws[bot_num][law_num]->getName();
}

unsigned int mt_Controller::getNumControlInputs(unsigned int bot_num, unsigned int law_num) const
{
    if(!validateBotAndLawNum(bot_num, law_num))
    {
        fprintf(stderr, "\t In mt_Controller::getNumControlInputs\n");
        return 0;
    }
    return m_vqpControlLaws[bot_num][law_num]->getNumControlInputs();
}

/* Note: It may be more efficient to pass in an already-allocated input
 * and return a reference to that same variable.  I've chosen not
 * to do that because we aren't experiencing any major performance
 * issues. */
mt_dVectorCollection_t mt_Controller::doControl(const mt_dVectorCollection_t& states,
                                                const mt_dVectorCollection_t& pre_inputs)
{
    mt_dVectorCollection_t control_out(getNumBots(), mt_CONTROLLER_EMPTY_VECTOR);
    for(unsigned int bot_num = 0; bot_num < getNumBots(); bot_num++)
    {
        control_out[bot_num] = calculateControlFor(bot_num,
                                                   states[bot_num],
                                                   pre_inputs[bot_num]);
    }
}

mt_dVector_t mt_Controller::calculateControlFor(unsigned int bot_num,
                                     const mt_dVector_t& state,
                                     const mt_dVector_t& u_in)
{
    mt_dVector_t u_to_now(0, 0.0);
    if(u_in.size() > 0)
    {
        u_to_now = u_in;
    }
    
    for(unsigned int law_num = 0; law_num < getNumLawsFor(bot_num); law_num++)
    {
        u_to_now = runControlLaw(bot_num, law_num, u_to_now, state);
    }
    return u_to_now;
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
                                          const mt_dVector_t& state,
                                          const mt_dVector_t& u_to_now)
{
    return m_vqpControlLaws[bot_num][law_num]->doControl(state, u_to_now);
}

bool mt_Controller::ensureControlLawProperties(unsigned int bot_num,
                                               const mt_ControlLaw* p_control_law) const
{
    if(bot_num >= getNumBots())
    {
        fprintf(stderr, "mt_Controller::ensureControlLawProperties error, "
                "requested robot number %d out of bounds "
                "(have %d)\n", bot_num, getNumBots());
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
