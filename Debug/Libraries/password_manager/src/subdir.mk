################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/password_manager/src/password_manager.c 

OBJS += \
./Libraries/password_manager/src/password_manager.o 

C_DEPS += \
./Libraries/password_manager/src/password_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/password_manager/src/%.o Libraries/password_manager/src/%.su Libraries/password_manager/src/%.cyclo: ../Libraries/password_manager/src/%.c Libraries/password_manager/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/duong/Desktop/Blue Pill/tkhtn_workspace/smart_door_lock/Libraries/keypad_driver/inc" -I"C:/Users/duong/Desktop/Blue Pill/tkhtn_workspace/smart_door_lock/Libraries/lcd_1602_i2c_driver/inc" -I"C:/Users/duong/Desktop/Blue Pill/tkhtn_workspace/smart_door_lock/Libraries/rfid_rc522_driver/inc" -I"C:/Users/duong/Desktop/Blue Pill/tkhtn_workspace/smart_door_lock/Libraries/misc/inc" -I"C:/Users/duong/Desktop/Blue Pill/tkhtn_workspace/smart_door_lock/Libraries/password_manager/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Libraries-2f-password_manager-2f-src

clean-Libraries-2f-password_manager-2f-src:
	-$(RM) ./Libraries/password_manager/src/password_manager.cyclo ./Libraries/password_manager/src/password_manager.d ./Libraries/password_manager/src/password_manager.o ./Libraries/password_manager/src/password_manager.su

.PHONY: clean-Libraries-2f-password_manager-2f-src
