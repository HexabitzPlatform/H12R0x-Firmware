################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/startup_stm32f091xc.s 

C_SRCS += \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_adc.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_dma.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_gpio.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_it.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_rtc.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_timers.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_uart.c 

OBJS += \
./H12R0/H12R0.o \
./H12R0/H12R0_adc.o \
./H12R0/H12R0_dma.o \
./H12R0/H12R0_gpio.o \
./H12R0/H12R0_it.o \
./H12R0/H12R0_rtc.o \
./H12R0/H12R0_timers.o \
./H12R0/H12R0_uart.o \
./H12R0/startup_stm32f091xc.o 

S_DEPS += \
./H12R0/startup_stm32f091xc.d 

C_DEPS += \
./H12R0/H12R0.d \
./H12R0/H12R0_adc.d \
./H12R0/H12R0_dma.d \
./H12R0/H12R0_gpio.d \
./H12R0/H12R0_it.d \
./H12R0/H12R0_rtc.d \
./H12R0/H12R0_timers.d \
./H12R0/H12R0_uart.d 


# Each subdirectory must supply rules for building sources it contributes
H12R0/H12R0.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H12R0/H12R0.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H12R0/H12R0_adc.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_adc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H12R0/H12R0_adc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H12R0/H12R0_dma.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H12R0/H12R0_dma.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H12R0/H12R0_gpio.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H12R0/H12R0_gpio.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H12R0/H12R0_it.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H12R0/H12R0_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H12R0/H12R0_rtc.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_rtc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H12R0/H12R0_rtc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H12R0/H12R0_timers.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H12R0/H12R0_timers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H12R0/H12R0_uart.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/H12R0_uart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H12R0/H12R0_uart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H12R0/startup_stm32f091xc.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/H12R0/startup_stm32f091xc.s
	arm-none-eabi-gcc -mcpu=cortex-m0 -g -c -x assembler-with-cpp -MMD -MP -MF"H12R0/startup_stm32f091xc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

