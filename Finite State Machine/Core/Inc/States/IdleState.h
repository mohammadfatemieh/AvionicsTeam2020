#ifndef TEMPLATED_IDLESTATE_H
#define TEMPLATED_IDLESTATE_H


#include "../FSM/state.h"
#include "../Transitions/ArmTransition.h"

class IdleState : public State {
    TRANSITIONS(new ArmTransition());

    ~IdleState() {
        END_STATE();
    }

    void onEnter() override;
    void onUpdate() override;
    void onExit() override;
};

#endif //TEMPLATED_IDLESTATE_H
