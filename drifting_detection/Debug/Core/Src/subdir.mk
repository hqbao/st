################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/optical_flow.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

CPP_SRCS += \
../Core/Src/Coarse2FineFlowWrapper.cpp \
../Core/Src/GaussianPyramid.cpp \
../Core/Src/OpticalFlow.cpp \
../Core/Src/Stochastic.cpp 

C_DEPS += \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/optical_flow.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 

OBJS += \
./Core/Src/Coarse2FineFlowWrapper.o \
./Core/Src/GaussianPyramid.o \
./Core/Src/OpticalFlow.o \
./Core/Src/Stochastic.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/optical_flow.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

CPP_DEPS += \
./Core/Src/Coarse2FineFlowWrapper.d \
./Core/Src/GaussianPyramid.d \
./Core/Src/OpticalFlow.d \
./Core/Src/Stochastic.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Coarse2FineFlowWrapper.d ./Core/Src/Coarse2FineFlowWrapper.o ./Core/Src/Coarse2FineFlowWrapper.su ./Core/Src/GaussianPyramid.d ./Core/Src/GaussianPyramid.o ./Core/Src/GaussianPyramid.su ./Core/Src/OpticalFlow.d ./Core/Src/OpticalFlow.o ./Core/Src/OpticalFlow.su ./Core/Src/Stochastic.d ./Core/Src/Stochastic.o ./Core/Src/Stochastic.su ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/optical_flow.d ./Core/Src/optical_flow.o ./Core/Src/optical_flow.su ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

