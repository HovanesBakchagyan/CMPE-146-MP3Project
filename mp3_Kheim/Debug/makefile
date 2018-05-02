################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include newlib/subdir.mk
-include _build/newlib/subdir.mk
-include _build/L5_Application/source/cmd_handlers/subdir.mk
-include _build/L5_Application/source/subdir.mk
-include _build/L5_Application/periodic_scheduler/subdir.mk
-include _build/L5_Application/examples/subdir.mk
-include _build/L5_Application/subdir.mk
-include _build/L4_IO/wireless/src/subdir.mk
-include _build/L4_IO/src/subdir.mk
-include _build/L4_IO/fat/option/subdir.mk
-include _build/L4_IO/fat/disk/subdir.mk
-include _build/L4_IO/fat/subdir.mk
-include _build/L3_Utils/tlm/src/subdir.mk
-include _build/L3_Utils/src/subdir.mk
-include _build/L2_Drivers/src/subdir.mk
-include _build/L2_Drivers/base/subdir.mk
-include _build/L1_FreeRTOS/src/subdir.mk
-include _build/L1_FreeRTOS/portable/no_mpu/subdir.mk
-include _build/L1_FreeRTOS/portable/subdir.mk
-include _build/L1_FreeRTOS/hooks/subdir.mk
-include _build/L0_LowLevel/source/subdir.mk
-include _build/subdir.mk
-include L5_Application/source/cmd_handlers/subdir.mk
-include L5_Application/source/subdir.mk
-include L5_Application/periodic_scheduler/subdir.mk
-include L5_Application/examples/subdir.mk
-include L5_Application/subdir.mk
-include L4_IO/wireless/src/subdir.mk
-include L4_IO/src/subdir.mk
-include L4_IO/fat/option/subdir.mk
-include L4_IO/fat/disk/subdir.mk
-include L4_IO/fat/subdir.mk
-include L3_Utils/tlm/src/subdir.mk
-include L3_Utils/src/subdir.mk
-include L2_Drivers/src/subdir.mk
-include L2_Drivers/base/subdir.mk
-include L1_FreeRTOS/src/subdir.mk
-include L1_FreeRTOS/portable/no_mpu/subdir.mk
-include L1_FreeRTOS/portable/mpu/subdir.mk
-include L1_FreeRTOS/portable/subdir.mk
-include L1_FreeRTOS/hooks/subdir.mk
-include L0_LowLevel/source/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
SSP_assignment.hex \

SECONDARY_SIZE += \
SSP_assignment.siz \


# All Target
all: SSP_assignment.elf secondary-outputs

# Tool invocations
SSP_assignment.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -Xlinker --gc-sections -Wl,-Map,"SSP_assignment.map" -o "SSP_assignment.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

SSP_assignment.hex: SSP_assignment.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "SSP_assignment.elf"  "SSP_assignment.hex"
	@echo 'Finished building: $@'
	@echo ' '

SSP_assignment.siz: SSP_assignment.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "SSP_assignment.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(C++_DEPS)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(CC_DEPS)$(SECONDARY_FLASH)$(CPP_DEPS)$(CXX_DEPS)$(C_UPPER_DEPS)$(S_UPPER_DEPS) SSP_assignment.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
