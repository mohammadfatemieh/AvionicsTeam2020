#ifndef TEMPLATED_ARMEDSTATE_H
#define TEMPLATED_ARMEDSTATE_H


#include "../FSM/state.h"

class ArmedState : public State {
public:
    void onUpdate() override;
};


#endif //TEMPLATED_ARMEDSTATE_H
