/*
 * ba_transition.cpp
 *
 *  Created on: Oct 27, 2019
 *      Author: cooli
 */


#include "transitions/ba_transition.h"
#include "states/a_state.h"

bool BATransition::isValid() {
	return true;
}

State* BATransition::getNextState() {
	return new AState();
}
