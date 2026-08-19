################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/User/Src/lcd_fonts.c \
../Drivers/User/Src/lcd_spi_154.c 

OBJS += \
./Drivers/User/Src/lcd_fonts.o \
./Drivers/User/Src/lcd_spi_154.o 

C_DEPS += \
./Drivers/User/Src/lcd_fonts.d \
./Drivers/User/Src/lcd_spi_154.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/User/Src/%.o Drivers/User/Src/%.su Drivers/User/Src/%.cyclo: ../Drivers/User/Src/%.c Drivers/User/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/User/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-User-2f-Src

clean-Drivers-2f-User-2f-Src:
	-$(RM) ./Drivers/User/Src/lcd_fonts.cyclo ./Drivers/User/Src/lcd_fonts.d ./Drivers/User/Src/lcd_fonts.o ./Drivers/User/Src/lcd_fonts.su ./Drivers/User/Src/lcd_spi_154.cyclo ./Drivers/User/Src/lcd_spi_154.d ./Drivers/User/Src/lcd_spi_154.o ./Drivers/User/Src/lcd_spi_154.su

.PHONY: clean-Drivers-2f-User-2f-Src

