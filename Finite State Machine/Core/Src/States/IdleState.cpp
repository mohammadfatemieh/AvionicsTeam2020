#include "States/IdleState.h"
#include "stm32f1xx_hal.h"

void IdleState::onEnter() {
	HAL_Delay(2500);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void IdleState::onUpdate() {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_Delay(500);
}

void IdleState::onExit() {
	HAL_Delay(2500);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
