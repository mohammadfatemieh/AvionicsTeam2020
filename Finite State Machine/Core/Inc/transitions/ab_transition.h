/*
 * abtransition.h
 *
 *  Created on: Oct 27, 2019
 *      Author: cooli
 */

#ifndef INC_TRANSITIONS_AB_TRANSITION_H_
#define INC_TRANSITIONS_AB_TRANSITION_H_


#include "fsm/transition.h"

class ABTransition : public Transition {
public:
    bool isValid() override;
    State* getNextState() override;
};


#endif /* INC_TRANSITIONS_AB_TRANSITION_H_ */
