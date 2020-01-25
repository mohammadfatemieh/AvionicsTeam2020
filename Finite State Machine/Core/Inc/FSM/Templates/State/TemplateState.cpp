#include <FSM/Templates/State/TemplateState.h>

namespace FSMTemplate {

void TemplateState::onEnter() {
	// This is called once, before onUpdate()
}

void TemplateState::onUpdate() {
	// This is looped while the state is active
}

void TemplateState::onExit() {
	// This is called after the last onUpdate()
}

} /* namespace FSMTemplate */
