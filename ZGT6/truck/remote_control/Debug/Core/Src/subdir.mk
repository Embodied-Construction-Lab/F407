################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/oled_ssd1306.c \
../Core/Src/pca9685.c \
../Core/Src/pwm_timing.c \
../Core/Src/servo_control.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/truck_control.c \
../Core/Src/truck_receiver.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/oled_ssd1306.o \
./Core/Src/pca9685.o \
./Core/Src/pwm_timing.o \
./Core/Src/servo_control.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/truck_control.o \
./Core/Src/truck_receiver.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/oled_ssd1306.d \
./Core/Src/pca9685.d \
./Core/Src/pwm_timing.d \
./Core/Src/servo_control.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/truck_control.d \
./Core/Src/truck_receiver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/User/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/oled_ssd1306.cyclo ./Core/Src/oled_ssd1306.d ./Core/Src/oled_ssd1306.o ./Core/Src/oled_ssd1306.su ./Core/Src/pca9685.cyclo ./Core/Src/pca9685.d ./Core/Src/pca9685.o ./Core/Src/pca9685.su ./Core/Src/pwm_timing.cyclo ./Core/Src/pwm_timing.d ./Core/Src/pwm_timing.o ./Core/Src/pwm_timing.su ./Core/Src/servo_control.cyclo ./Core/Src/servo_control.d ./Core/Src/servo_control.o ./Core/Src/servo_control.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/truck_control.cyclo ./Core/Src/truck_control.d ./Core/Src/truck_control.o ./Core/Src/truck_control.su ./Core/Src/truck_receiver.cyclo ./Core/Src/truck_receiver.d ./Core/Src/truck_receiver.o ./Core/Src/truck_receiver.su

.PHONY: clean-Core-2f-Src

