################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/lcd_1602_i2c_driver/src/lcd_1602_i2c_driver.c 

OBJS += \
./Libraries/lcd_1602_i2c_driver/src/lcd_1602_i2c_driver.o 

C_DEPS += \
./Libraries/lcd_1602_i2c_driver/src/lcd_1602_i2c_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/lcd_1602_i2c_driver/src/%.o Libraries/lcd_1602_i2c_driver/src/%.su Libraries/lcd_1602_i2c_driver/src/%.cyclo: ../Libraries/lcd_1602_i2c_driver/src/%.c Libraries/lcd_1602_i2c_driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/duong/Desktop/Blue Pill/tkhtn_workspace/smart_door_lock/Libraries/keypad_driver/inc" -I"C:/Users/duong/Desktop/Blue Pill/tkhtn_workspace/smart_door_lock/Libraries/lcd_1602_i2c_driver/inc" -I"C:/Users/duong/Desktop/Blue Pill/tkhtn_workspace/smart_door_lock/Libraries/rfid_rc522_driver/inc" -I"C:/Users/duong/Desktop/Blue Pill/tkhtn_workspace/smart_door_lock/Libraries/misc/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Libraries-2f-lcd_1602_i2c_driver-2f-src

clean-Libraries-2f-lcd_1602_i2c_driver-2f-src:
	-$(RM) ./Libraries/lcd_1602_i2c_driver/src/lcd_1602_i2c_driver.cyclo ./Libraries/lcd_1602_i2c_driver/src/lcd_1602_i2c_driver.d ./Libraries/lcd_1602_i2c_driver/src/lcd_1602_i2c_driver.o ./Libraries/lcd_1602_i2c_driver/src/lcd_1602_i2c_driver.su

.PHONY: clean-Libraries-2f-lcd_1602_i2c_driver-2f-src

