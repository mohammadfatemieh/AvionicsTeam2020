#ifndef INC_FSM_TRANSITION_H_
#define INC_FSM_TRANSITION_H_


#include <cstdint>
#include "state.h"

/** Transitions
 * \breif Defines the conditions for going to a given state
 */
class Transition {
public:
	virtual ~Transition() {}

	virtual bool isValid() = 0;
	virtual State* getNextState() = 0;
};


#endif /* INC_FSM_TRANSITION_H_ */
