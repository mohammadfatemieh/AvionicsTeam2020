#ifndef PROPULSE_STATEMACHINE_H
#define PROPULSE_STATEMACHINE_H


#include <FSM/state.h>

/** STATE MACHINE
 * \breif The brains of the operations
 * @tparam I - Initial state
 */

template <typename I>
class StateMachine {
private:
    /// Pointer to the current active state
    State* activeState;

public:
    /// Set the active state to a new instance of the specified initial state, and call the first callback (onEnter)
    StateMachine() : activeState(new I()) {
        activeState->onEnter();
    }

    /// Clean up active state pointer
    ~StateMachine() {
        delete activeState;
    }

    /// ...
    /// If a valid transition exits, call appropriate callbacks and change active state
    /// After checking for transitions, call the main update() function of the active state
    void update() {
        State* nextState;
        if(activeState->getValidTransition(nextState)) {
            activeState->onExit();
            nextState->onEnter();

            delete activeState;
            activeState = nextState;
        }

        activeState->onUpdate();
    }
};

#endif //PROPULSE_STATEMACHINE_H
