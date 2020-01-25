#ifndef TEMPLATED_TRANSITION_H
#define TEMPLATED_TRANSITION_H


#include <FSM/state.h>

/** TRANSITION
 * \breif Defines the conditions for going to a given state
 */

class Transition {
public:
    /// Must have a virtual destructor
    virtual ~Transition() {}

    virtual bool isValid() = 0;
    virtual State* getNextState() = 0;
};


template <typename S>
class TransitionTo : public Transition {
public:
    virtual State* getNextState() override {
        return new S();
    };
};


#endif //TEMPLATED_TRANSITION_H
