################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MMR_CAN/Src/mmr_can.c \
../Drivers/MMR_CAN/Src/mmr_can_events.c \
../Drivers/MMR_CAN/Src/mmr_can_header.c \
../Drivers/MMR_CAN/Src/mmr_can_message_id.c \
../Drivers/MMR_CAN/Src/mmr_can_receive.c \
../Drivers/MMR_CAN/Src/mmr_can_scs_entries.c \
../Drivers/MMR_CAN/Src/mmr_can_scs_manager.c \
../Drivers/MMR_CAN/Src/mmr_can_send.c 

OBJS += \
./Drivers/MMR_CAN/Src/mmr_can.o \
./Drivers/MMR_CAN/Src/mmr_can_events.o \
./Drivers/MMR_CAN/Src/mmr_can_header.o \
./Drivers/MMR_CAN/Src/mmr_can_message_id.o \
./Drivers/MMR_CAN/Src/mmr_can_receive.o \
./Drivers/MMR_CAN/Src/mmr_can_scs_entries.o \
./Drivers/MMR_CAN/Src/mmr_can_scs_manager.o \
./Drivers/MMR_CAN/Src/mmr_can_send.o 

C_DEPS += \
./Drivers/MMR_CAN/Src/mmr_can.d \
./Drivers/MMR_CAN/Src/mmr_can_events.d \
./Drivers/MMR_CAN/Src/mmr_can_header.d \
./Drivers/MMR_CAN/Src/mmr_can_message_id.d \
./Drivers/MMR_CAN/Src/mmr_can_receive.d \
./Drivers/MMR_CAN/Src/mmr_can_scs_entries.d \
./Drivers/MMR_CAN/Src/mmr_can_scs_manager.d \
./Drivers/MMR_CAN/Src/mmr_can_send.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MMR_CAN/Src/%.o Drivers/MMR_CAN/Src/%.su: ../Drivers/MMR_CAN/Src/%.c Drivers/MMR_CAN/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Drivers/MMR_CAN/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MMR_CAN-2f-Src

clean-Drivers-2f-MMR_CAN-2f-Src:
	-$(RM) ./Drivers/MMR_CAN/Src/mmr_can.d ./Drivers/MMR_CAN/Src/mmr_can.o ./Drivers/MMR_CAN/Src/mmr_can.su ./Drivers/MMR_CAN/Src/mmr_can_events.d ./Drivers/MMR_CAN/Src/mmr_can_events.o ./Drivers/MMR_CAN/Src/mmr_can_events.su ./Drivers/MMR_CAN/Src/mmr_can_header.d ./Drivers/MMR_CAN/Src/mmr_can_header.o ./Drivers/MMR_CAN/Src/mmr_can_header.su ./Drivers/MMR_CAN/Src/mmr_can_message_id.d ./Drivers/MMR_CAN/Src/mmr_can_message_id.o ./Drivers/MMR_CAN/Src/mmr_can_message_id.su ./Drivers/MMR_CAN/Src/mmr_can_receive.d ./Drivers/MMR_CAN/Src/mmr_can_receive.o ./Drivers/MMR_CAN/Src/mmr_can_receive.su ./Drivers/MMR_CAN/Src/mmr_can_scs_entries.d ./Drivers/MMR_CAN/Src/mmr_can_scs_entries.o ./Drivers/MMR_CAN/Src/mmr_can_scs_entries.su ./Drivers/MMR_CAN/Src/mmr_can_scs_manager.d ./Drivers/MMR_CAN/Src/mmr_can_scs_manager.o ./Drivers/MMR_CAN/Src/mmr_can_scs_manager.su ./Drivers/MMR_CAN/Src/mmr_can_send.d ./Drivers/MMR_CAN/Src/mmr_can_send.o ./Drivers/MMR_CAN/Src/mmr_can_send.su

.PHONY: clean-Drivers-2f-MMR_CAN-2f-Src

