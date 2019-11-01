/*
 * astate.h
 *
 *  Created on: Oct 27, 2019
 *      Author: cooli
 */

#ifndef INC_STATES_A_STATE_H_
#define INC_STATES_A_STATE_H_


#include "main.h"
#include "fsm/state.h"
#include "transitions/ab_transition.h"

class AState : public State {
	TRANSITIONS(new ABTransition())

    ~AState() override {
        END_STATE()
    }

    void onEnter() override {
    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	HAL_Delay(3000);
    };
};


#endif /* INC_STATES_A_STATE_H_ */
