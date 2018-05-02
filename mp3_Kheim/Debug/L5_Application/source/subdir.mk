################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../L5_Application/source/high_level_init.cpp \
../L5_Application/source/remote.cpp \
../L5_Application/source/terminal.cpp 

OBJS += \
./L5_Application/source/high_level_init.o \
./L5_Application/source/remote.o \
./L5_Application/source/terminal.o 

CPP_DEPS += \
./L5_Application/source/high_level_init.d \
./L5_Application/source/remote.d \
./L5_Application/source/terminal.d 


# Each subdirectory must supply rules for building sources it contributes
L5_Application/source/%.o: ../L5_Application/source/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -std=gnu++11 -fabi-version=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


