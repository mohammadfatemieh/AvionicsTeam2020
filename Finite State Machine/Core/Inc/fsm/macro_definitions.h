/**\def Macro Definitions
 * \breif Defines helpful functions for working with the FSM.
 *
 *
 */

#ifndef INC_FSM_MACRO_DEFINITIONS_H_
#define INC_FSM_MACRO_DEFINITIONS_H_


// Alternatively use C++ std::tuple or C __VA_ARGS__ dual macros, but this is cleanest pure C solution
// From chromium | https://stackoverflow.com/questions/4415524/common-array-length-macro-for-c
#define LENGTH_OF(x) ((sizeof(x)/sizeof(0[x])) / ((int)(!(sizeof(x) % sizeof(0[x]))))) // NOLINT

/** \def TRANSITIONS
 *	\breif Define what transitions
 */
#define TRANSITIONS(l...) public:						\
	Transition* transitions[LENGTH_OF(l)] = {l};		\
	bool getValidTransition(State* &next) override {	\
		for(uint8_t i = 0; i < LENGTH_OF(l); i++){		\
			if(transitions[i]->isValid()){				\
				next = transitions[i]->getNextState();	\
				return true;							\
			}											\
		}												\
		return false;									\
	}

/** \def END_STATE()
 *  Frees the memory used by the states transitions
 */
#define END_STATE()											\
	for(uint8_t i = 0; i < LENGTH_OF(transitions); i++) {	\
		delete transitions[i];								\
	}


#endif /* INC_FSM_MACRO_DEFINITIONS_H_ */
