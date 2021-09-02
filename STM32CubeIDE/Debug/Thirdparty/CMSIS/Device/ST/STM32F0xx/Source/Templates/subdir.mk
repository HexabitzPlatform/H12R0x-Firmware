################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c 

OBJS += \
./Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.o 

C_DEPS += \
./Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

