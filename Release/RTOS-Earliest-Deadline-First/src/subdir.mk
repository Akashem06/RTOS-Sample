################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RTOS-Earliest-Deadline-First/src/mutex.c \
../RTOS-Earliest-Deadline-First/src/queue.c \
../RTOS-Earliest-Deadline-First/src/scheduler.c 

OBJS += \
./RTOS-Earliest-Deadline-First/src/mutex.o \
./RTOS-Earliest-Deadline-First/src/queue.o \
./RTOS-Earliest-Deadline-First/src/scheduler.o 

C_DEPS += \
./RTOS-Earliest-Deadline-First/src/mutex.d \
./RTOS-Earliest-Deadline-First/src/queue.d \
./RTOS-Earliest-Deadline-First/src/scheduler.d 


# Each subdirectory must supply rules for building sources it contributes
RTOS-Earliest-Deadline-First/src/%.o RTOS-Earliest-Deadline-First/src/%.su RTOS-Earliest-Deadline-First/src/%.cyclo: ../RTOS-Earliest-Deadline-First/src/%.c RTOS-Earliest-Deadline-First/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../RTOS-Earliest-Deadline-First/inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RTOS-2d-Earliest-2d-Deadline-2d-First-2f-src

clean-RTOS-2d-Earliest-2d-Deadline-2d-First-2f-src:
	-$(RM) ./RTOS-Earliest-Deadline-First/src/mutex.cyclo ./RTOS-Earliest-Deadline-First/src/mutex.d ./RTOS-Earliest-Deadline-First/src/mutex.o ./RTOS-Earliest-Deadline-First/src/mutex.su ./RTOS-Earliest-Deadline-First/src/queue.cyclo ./RTOS-Earliest-Deadline-First/src/queue.d ./RTOS-Earliest-Deadline-First/src/queue.o ./RTOS-Earliest-Deadline-First/src/queue.su ./RTOS-Earliest-Deadline-First/src/scheduler.cyclo ./RTOS-Earliest-Deadline-First/src/scheduler.d ./RTOS-Earliest-Deadline-First/src/scheduler.o ./RTOS-Earliest-Deadline-First/src/scheduler.su

.PHONY: clean-RTOS-2d-Earliest-2d-Deadline-2d-First-2f-src

