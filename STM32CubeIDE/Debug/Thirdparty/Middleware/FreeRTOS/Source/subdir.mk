################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/croutine.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/event_groups.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/list.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/queue.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/tasks.c \
C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/timers.c 

OBJS += \
./Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.o \
./Thirdparty/Middleware/FreeRTOS/Source/croutine.o \
./Thirdparty/Middleware/FreeRTOS/Source/event_groups.o \
./Thirdparty/Middleware/FreeRTOS/Source/list.o \
./Thirdparty/Middleware/FreeRTOS/Source/queue.o \
./Thirdparty/Middleware/FreeRTOS/Source/tasks.o \
./Thirdparty/Middleware/FreeRTOS/Source/timers.o 

C_DEPS += \
./Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.d \
./Thirdparty/Middleware/FreeRTOS/Source/croutine.d \
./Thirdparty/Middleware/FreeRTOS/Source/event_groups.d \
./Thirdparty/Middleware/FreeRTOS/Source/list.d \
./Thirdparty/Middleware/FreeRTOS/Source/queue.d \
./Thirdparty/Middleware/FreeRTOS/Source/tasks.d \
./Thirdparty/Middleware/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/croutine.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/croutine.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/croutine.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/event_groups.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/event_groups.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/event_groups.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/list.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/list.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/list.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/queue.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/queue.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/queue.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/tasks.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/tasks.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/tasks.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/timers.o: C:/Users/User/Desktop/hexa_projects/Modules_firmware/H12R0x-Firmware-master/Thirdparty/Middleware/FreeRTOS/Source/timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DSTM32 -DSTM32F0 -DSTM32F091CBUx -DH12R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../Inc -I../../H12R0 -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/timers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

