#ifndef PROPULSE_STATE_H
#define PROPULSE_STATE_H


/** STATE
 * \breif The logic of each rocket phase, separated from the transitional logic
 */

#include "../FSM/macros.h"

class State {
public:
    /// Use the destructor to clean up transitions from memory, preventing memory leaks.
    /// For implementation see the macro END_STATE().
    virtual ~State() { };

    // Transition Logic
    /// Iterates all the states transitions, and returns the next state if a valid transition is found.
    /// For implementation see the macro TRANSITIONS(l...).
    virtual bool getValidTransition(State* &next) {
        return false;
    };

    // Transition Callbacks
    /// Called before the state is entered, before the first update().
    virtual void onEnter() {};
    /// Called after the state is exited, after the last update().
    virtual void onExit() {};

    // Main Logic
    /// Called every tick of the flight computer, while the state is active.
    virtual void onUpdate() {};
};


#endif //PROPULSE_STATE_H