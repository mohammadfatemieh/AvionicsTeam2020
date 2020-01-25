################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Adafruit_GPS_STM32_Porting.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Adafruit_GPS_STM32_Porting.list \


# All Target
all: Adafruit_GPS_STM32_Porting.elf secondary-outputs

# Tool invocations
Adafruit_GPS_STM32_Porting.elf: $(OBJS) $(USER_OBJS) /media/data/Propulse/AvionicsTeam2020/Prototyping/Adafruit_GPS_STM32_Porting/STM32F103C8TX_FLASH.ld
	arm-none-eabi-g++ -o "Adafruit_GPS_STM32_Porting.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/media/data/Propulse/AvionicsTeam2020/Prototyping/Adafruit_GPS_STM32_Porting/STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Adafruit_GPS_STM32_Porting.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Adafruit_GPS_STM32_Porting.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Adafruit_GPS_STM32_Porting.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
