/*
 * bstate.h
 *
 *  Created on: Oct 27, 2019
 *      Author: cooli
 */

#ifndef INC_STATES_B_STATE_H_
#define INC_STATES_B_STATE_H_


#include "main.h"
#include "fsm/state.h"
#include "transitions/ba_transition.h"

class BState : public State {
	TRANSITIONS(new BATransition())

    ~BState() override {
        END_STATE()
    }

    void onEnter() override {
    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	HAL_Delay(500);
    };
};


#endif /* INC_STATES_B_STATE_H_ */
