#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_


#include "state.h"

/** Finite State Machine
 * Lmao
 */

template <typename S>
class StateMachine {
	State* activeState;

public: /* Initialization & Deconstruction */
	StateMachine() : activeState(new S()) {
		activeState->onEnter();
	}

	~StateMachine() {
		delete activeState;
	}

public: /* Main Functionality */
	void update() {
		State* next;
		if (activeState->getValidTransition(next)) {
			activeState->onExit();
			next->onEnter();

			delete activeState;
			activeState = next;
		}

		activeState->onUpdate();
	}
};


#endif /* INC_STATEMACHINE_H_ */
