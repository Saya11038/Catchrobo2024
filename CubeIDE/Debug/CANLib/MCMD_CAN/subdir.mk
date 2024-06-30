################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CANLib/MCMD_CAN/MCMD_CAN.c 

OBJS += \
./CANLib/MCMD_CAN/MCMD_CAN.o 

C_DEPS += \
./CANLib/MCMD_CAN/MCMD_CAN.d 


# Each subdirectory must supply rules for building sources it contributes
CANLib/MCMD_CAN/%.o CANLib/MCMD_CAN/%.su CANLib/MCMD_CAN/%.cyclo: ../CANLib/MCMD_CAN/%.c CANLib/MCMD_CAN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"/home/saya/STM32CubeIDE/workspace_1.15.1/Catchrobo_2024/CANLib/CAN_Main" -I"/home/saya/STM32CubeIDE/workspace_1.15.1/Catchrobo_2024/CANLib/Defines/Inc" -I"/home/saya/STM32CubeIDE/workspace_1.15.1/Catchrobo_2024/CANLib/MCMD_CAN" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CANLib-2f-MCMD_CAN

clean-CANLib-2f-MCMD_CAN:
	-$(RM) ./CANLib/MCMD_CAN/MCMD_CAN.cyclo ./CANLib/MCMD_CAN/MCMD_CAN.d ./CANLib/MCMD_CAN/MCMD_CAN.o ./CANLib/MCMD_CAN/MCMD_CAN.su

.PHONY: clean-CANLib-2f-MCMD_CAN

