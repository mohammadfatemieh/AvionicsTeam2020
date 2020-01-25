#include "Transitions/ArmTransition.h"

bool ArmTransition::isValid() {
	return iteration-- == 0;
}
