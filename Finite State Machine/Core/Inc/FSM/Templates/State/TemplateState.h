#ifndef INC_FSM_TEMPLATES_STATE_TEMPLATESTATE_H_
#define INC_FSM_TEMPLATES_STATE_TEMPLATESTATE_H_

#include "../../state.h"
#include "../Transition/TemplateTransition.h"

// Namespaced so template files cannot conflict with real files
namespace FSMTemplate {

class TemplateState : public State {
	// -- Transitions

	// Supports up to 64 comma separated transitions
	TRANSITIONS(new TemplateTransition());

	// END_STATE() must be used in destructor if and only if TRANSITIONS() is used. This is to prevent memory leaks.
	~TemplateState() {
		END_STATE();
	}

	// -- Overrides
	// A state does not require any overrides to be defined, you only need to define the callbacks you want to use.

	// Called once before the first onUpdate().
	void onEnter() override;

	// Called continuously while the current state is the active state.
	void onUpdate() override;

	// Called after the last onUpdate(), before the next states onEnter().
	void onExit() override;
};

} /* namespace FSMTemplate */

#endif /* INC_FSM_TEMPLATES_STATE_TEMPLATESTATE_H_ */
