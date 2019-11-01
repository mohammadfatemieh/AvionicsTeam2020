/*
 * ab_transition.cpp
 *
 *  Created on: Oct 27, 2019
 *      Author: cooli
 */


#include "transitions/ab_transition.h"
#include "states/b_state.h"

bool ABTransition::isValid() {
	return true;
}

State* ABTransition::getNextState() {
	return new BState();
}
