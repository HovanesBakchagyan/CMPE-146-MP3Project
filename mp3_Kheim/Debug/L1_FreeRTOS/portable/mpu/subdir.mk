################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../L1_FreeRTOS/portable/mpu/port.c 

OBJS += \
./L1_FreeRTOS/portable/mpu/port.o 

C_DEPS += \
./L1_FreeRTOS/portable/mpu/port.d 


# Each subdirectory must supply rules for building sources it contributes
L1_FreeRTOS/portable/mpu/%.o: ../L1_FreeRTOS/portable/mpu/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


