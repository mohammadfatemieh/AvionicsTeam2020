/*
 * transition.h
 *
 *  Created on: Oct 27, 2019
 *      Author: cooli
 */

#ifndef INC_FSM_TRANSITION_H_
#define INC_FSM_TRANSITION_H_


#include <cstdint>
#include "state.h"

/** Transition */
//template <typename S>
class Transition {
public:
	virtual ~Transition() {}

	virtual bool isValid() = 0;
	virtual State* getNextState() = 0;
};

/** C Macro Definitions */
// Alternatively use C++ std::tuple or C __VA_ARGS__ dual macros, but this is cleanest pure C solution
// From chromium | https://stackoverflow.com/questions/4415524/common-array-length-macro-for-c
#define LENGTH_OF(x) ((sizeof(x)/sizeof(0[x])) / ((int)(!(sizeof(x) % sizeof(0[x]))))) // NOLINT

#define END_STATE()\
	for(uint8_t i = 0; i < LENGTH_OF(transitions); i++) {\
		delete transitions[i];\
	}\

#define TRANSITIONS(l...) public:\
	Transition* transitions[LENGTH_OF(l)] = {l};\
	bool getValidTransition(State* &next) override {\
		for(uint8_t i = 0; i < LENGTH_OF(l); i++){\
			if(transitions[i]->isValid()){\
				next = transitions[i]->getNextState();\
				return true;\
			}\
		}\
		return false;\
	}


#endif /* INC_FSM_TRANSITION_H_ */
