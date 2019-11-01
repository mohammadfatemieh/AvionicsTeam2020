/*
 * state.h
 *
 *  Created on: Oct 27, 2019
 *      Author: cooli
 */

#ifndef INC_FSM_STATE_H_
#define INC_FSM_STATE_H_


/** State */
class State {
public:
    // Clean Up Transitions
    virtual ~State() { };

    // Transition Logic
    virtual bool getValidTransition(State* &next) = 0;

    // Transition Callbacks
    virtual void onExit() {};
    virtual void onEnter() {};

    // Main Logic
    virtual void onUpdate() {};
};


#endif /* INC_FSM_STATE_H_ */
