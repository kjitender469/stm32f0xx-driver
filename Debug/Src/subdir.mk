################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/USART_Tx_testing.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/USART_Tx_testing.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/USART_Tx_testing.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/USART_Tx_testing.o: ../Src/USART_Tx_testing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F030R8Tx -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F0308_DISCO -c -I../Inc -I"/home/jitu/STM32CubeIDE/stm32_workspace_1.0.2/stm32f0xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/USART_Tx_testing.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F030R8Tx -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F0308_DISCO -c -I../Inc -I"/home/jitu/STM32CubeIDE/stm32_workspace_1.0.2/stm32f0xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F030R8Tx -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F0308_DISCO -c -I../Inc -I"/home/jitu/STM32CubeIDE/stm32_workspace_1.0.2/stm32f0xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

