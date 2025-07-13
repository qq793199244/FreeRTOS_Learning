################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/master_to_load.c \
../BSP/master_to_motordriver.c \
../BSP/modbus_rtu_base.c \
../BSP/slave_to_plc.c 

OBJS += \
./BSP/master_to_load.o \
./BSP/master_to_motordriver.o \
./BSP/modbus_rtu_base.o \
./BSP/slave_to_plc.o 

C_DEPS += \
./BSP/master_to_load.d \
./BSP/master_to_motordriver.d \
./BSP/modbus_rtu_base.d \
./BSP/slave_to_plc.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o BSP/%.su BSP/%.cyclo: ../BSP/%.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../BSP -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP

clean-BSP:
	-$(RM) ./BSP/master_to_load.cyclo ./BSP/master_to_load.d ./BSP/master_to_load.o ./BSP/master_to_load.su ./BSP/master_to_motordriver.cyclo ./BSP/master_to_motordriver.d ./BSP/master_to_motordriver.o ./BSP/master_to_motordriver.su ./BSP/modbus_rtu_base.cyclo ./BSP/modbus_rtu_base.d ./BSP/modbus_rtu_base.o ./BSP/modbus_rtu_base.su ./BSP/slave_to_plc.cyclo ./BSP/slave_to_plc.d ./BSP/slave_to_plc.o ./BSP/slave_to_plc.su

.PHONY: clean-BSP

