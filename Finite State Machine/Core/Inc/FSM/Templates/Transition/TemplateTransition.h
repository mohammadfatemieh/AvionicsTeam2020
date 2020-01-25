#ifndef INC_FSM_TEMPLATES_TRANSITION_TEMPLATETRANSITION_H_
#define INC_FSM_TEMPLATES_TRANSITION_TEMPLATETRANSITION_H_


#include "../../transition.h"
#include "../State/TemplateState.h"

// Namespaced so template files cannot conflict with real files
namespace FSMTemplate {

	class TemplateTransition : public TransitionTo<TemplateState> {
	public:
		// Checked continuously, when true the FSM will transition to the <State> defined in the extended class, ie. TemplateState in this case.
		bool isValid() override;
	};

} /* namespace FSMTemplate */

#endif /* INC_FSM_TEMPLATES_TRANSITION_TEMPLATETRANSITION_H_ */
