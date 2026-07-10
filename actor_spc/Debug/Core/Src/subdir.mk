################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/arm_angle.c \
../Core/Src/dwj_reader.c \
../Core/Src/ftepc_rs485.c \
../Core/Src/homing_controller.c \
../Core/Src/imu_oled_format.c \
../Core/Src/imu_parser.c \
../Core/Src/joystick_servo_map.c \
../Core/Src/main.c \
../Core/Src/motion_telemetry.c \
../Core/Src/oled_ssd1306.c \
../Core/Src/pca9685.c \
../Core/Src/pid_controller.c \
../Core/Src/pwm_timing.c \
../Core/Src/safety_limits.c \
../Core/Src/servo_control.c \
../Core/Src/servo_debug.c \
../Core/Src/speed_command_preprocess.c \
../Core/Src/speed_command_receiver.c \
../Core/Src/speed_feedback.c \
../Core/Src/status_led.c \
../Core/Src/stick_receiver.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/arm_angle.o \
./Core/Src/dwj_reader.o \
./Core/Src/ftepc_rs485.o \
./Core/Src/homing_controller.o \
./Core/Src/imu_oled_format.o \
./Core/Src/imu_parser.o \
./Core/Src/joystick_servo_map.o \
./Core/Src/main.o \
./Core/Src/motion_telemetry.o \
./Core/Src/oled_ssd1306.o \
./Core/Src/pca9685.o \
./Core/Src/pid_controller.o \
./Core/Src/pwm_timing.o \
./Core/Src/safety_limits.o \
./Core/Src/servo_control.o \
./Core/Src/servo_debug.o \
./Core/Src/speed_command_preprocess.o \
./Core/Src/speed_command_receiver.o \
./Core/Src/speed_feedback.o \
./Core/Src/status_led.o \
./Core/Src/stick_receiver.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/arm_angle.d \
./Core/Src/dwj_reader.d \
./Core/Src/ftepc_rs485.d \
./Core/Src/homing_controller.d \
./Core/Src/imu_oled_format.d \
./Core/Src/imu_parser.d \
./Core/Src/joystick_servo_map.d \
./Core/Src/main.d \
./Core/Src/motion_telemetry.d \
./Core/Src/oled_ssd1306.d \
./Core/Src/pca9685.d \
./Core/Src/pid_controller.d \
./Core/Src/pwm_timing.d \
./Core/Src/safety_limits.d \
./Core/Src/servo_control.d \
./Core/Src/servo_debug.d \
./Core/Src/speed_command_preprocess.d \
./Core/Src/speed_command_receiver.d \
./Core/Src/speed_feedback.d \
./Core/Src/status_led.d \
./Core/Src/stick_receiver.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/arm_angle.cyclo ./Core/Src/arm_angle.d ./Core/Src/arm_angle.o ./Core/Src/arm_angle.su ./Core/Src/dwj_reader.cyclo ./Core/Src/dwj_reader.d ./Core/Src/dwj_reader.o ./Core/Src/dwj_reader.su ./Core/Src/ftepc_rs485.cyclo ./Core/Src/ftepc_rs485.d ./Core/Src/ftepc_rs485.o ./Core/Src/ftepc_rs485.su ./Core/Src/homing_controller.cyclo ./Core/Src/homing_controller.d ./Core/Src/homing_controller.o ./Core/Src/homing_controller.su ./Core/Src/imu_oled_format.cyclo ./Core/Src/imu_oled_format.d ./Core/Src/imu_oled_format.o ./Core/Src/imu_oled_format.su ./Core/Src/imu_parser.cyclo ./Core/Src/imu_parser.d ./Core/Src/imu_parser.o ./Core/Src/imu_parser.su ./Core/Src/joystick_servo_map.cyclo ./Core/Src/joystick_servo_map.d ./Core/Src/joystick_servo_map.o ./Core/Src/joystick_servo_map.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motion_telemetry.cyclo ./Core/Src/motion_telemetry.d ./Core/Src/motion_telemetry.o ./Core/Src/motion_telemetry.su ./Core/Src/oled_ssd1306.cyclo ./Core/Src/oled_ssd1306.d ./Core/Src/oled_ssd1306.o ./Core/Src/oled_ssd1306.su ./Core/Src/pca9685.cyclo ./Core/Src/pca9685.d ./Core/Src/pca9685.o ./Core/Src/pca9685.su ./Core/Src/pid_controller.cyclo ./Core/Src/pid_controller.d ./Core/Src/pid_controller.o ./Core/Src/pid_controller.su ./Core/Src/pwm_timing.cyclo ./Core/Src/pwm_timing.d ./Core/Src/pwm_timing.o ./Core/Src/pwm_timing.su ./Core/Src/safety_limits.cyclo ./Core/Src/safety_limits.d ./Core/Src/safety_limits.o ./Core/Src/safety_limits.su ./Core/Src/servo_control.cyclo ./Core/Src/servo_control.d ./Core/Src/servo_control.o ./Core/Src/servo_control.su ./Core/Src/servo_debug.cyclo ./Core/Src/servo_debug.d ./Core/Src/servo_debug.o ./Core/Src/servo_debug.su ./Core/Src/speed_command_preprocess.cyclo ./Core/Src/speed_command_preprocess.d ./Core/Src/speed_command_preprocess.o ./Core/Src/speed_command_preprocess.su ./Core/Src/speed_command_receiver.cyclo ./Core/Src/speed_command_receiver.d ./Core/Src/speed_command_receiver.o ./Core/Src/speed_command_receiver.su ./Core/Src/speed_feedback.cyclo ./Core/Src/speed_feedback.d ./Core/Src/speed_feedback.o ./Core/Src/speed_feedback.su ./Core/Src/status_led.cyclo ./Core/Src/status_led.d ./Core/Src/status_led.o ./Core/Src/status_led.su ./Core/Src/stick_receiver.cyclo ./Core/Src/stick_receiver.d ./Core/Src/stick_receiver.o ./Core/Src/stick_receiver.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

