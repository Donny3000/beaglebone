################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_SIZE += \
gpe-app.siz \


# All Target
all: gpe-app.elf secondary-outputs

# Tool invocations
gpe-app.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Linux GCC C Linker'
	arm-linux-gnueabihf-gcc -L/usr/local/xenomai/lib -Wl,-Map,gpe-app.map -mcpu=cortex-a8 -mfloat-abi=hard -mfpu=neon -g3 -gdwarf-2 -o "gpe-app.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

gpe-app.siz: gpe-app.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Print Size'
	arm-linux-gnueabihf-size  --format=berkeley gpe-app.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(S_UPPER_DEPS)$(C_DEPS)$(SECONDARY_SIZE)$(ASM_DEPS)$(OBJS) gpe-app.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
