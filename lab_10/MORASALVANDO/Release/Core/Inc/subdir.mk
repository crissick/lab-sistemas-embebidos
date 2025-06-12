################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/ssd1306.c \
../Core/Inc/ssd1306_fonts.c \
../Core/Inc/ssd1306_tests.c 

OBJS += \
./Core/Inc/ssd1306.o \
./Core/Inc/ssd1306_fonts.o \
./Core/Inc/ssd1306_tests.o 

C_DEPS += \
./Core/Inc/ssd1306.d \
./Core/Inc/ssd1306_fonts.d \
./Core/Inc/ssd1306_tests.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/ssd1306.cyclo ./Core/Inc/ssd1306.d ./Core/Inc/ssd1306.o ./Core/Inc/ssd1306.su ./Core/Inc/ssd1306_fonts.cyclo ./Core/Inc/ssd1306_fonts.d ./Core/Inc/ssd1306_fonts.o ./Core/Inc/ssd1306_fonts.su ./Core/Inc/ssd1306_tests.cyclo ./Core/Inc/ssd1306_tests.d ./Core/Inc/ssd1306_tests.o ./Core/Inc/ssd1306_tests.su

.PHONY: clean-Core-2f-Inc

