#ifndef TEMPLATED_ARMTRANSITION_H
#define TEMPLATED_ARMTRANSITION_H


#include "../FSM/transition.h"
#include "../States/ArmedState.h"

class ArmTransition : public TransitionTo<ArmedState> {
private:
    int iteration = 3;

public:
    bool isValid() override;
};


#endif //TEMPLATED_ARMTRANSITION_H
