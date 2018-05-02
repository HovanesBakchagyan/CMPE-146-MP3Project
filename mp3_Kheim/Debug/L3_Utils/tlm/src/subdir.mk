################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../L3_Utils/tlm/src/c_tlm_binary.c \
../L3_Utils/tlm/src/c_tlm_comp.c \
../L3_Utils/tlm/src/c_tlm_stream.c \
../L3_Utils/tlm/src/c_tlm_var.c 

OBJS += \
./L3_Utils/tlm/src/c_tlm_binary.o \
./L3_Utils/tlm/src/c_tlm_comp.o \
./L3_Utils/tlm/src/c_tlm_stream.o \
./L3_Utils/tlm/src/c_tlm_var.o 

C_DEPS += \
./L3_Utils/tlm/src/c_tlm_binary.d \
./L3_Utils/tlm/src/c_tlm_comp.d \
./L3_Utils/tlm/src/c_tlm_stream.d \
./L3_Utils/tlm/src/c_tlm_var.d 


# Each subdirectory must supply rules for building sources it contributes
L3_Utils/tlm/src/%.o: ../L3_Utils/tlm/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


