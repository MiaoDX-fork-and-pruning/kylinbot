################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/bj/workspace/eclipse/stm32/kylinbot/srv/src/can_srv.c \
/home/bj/workspace/eclipse/stm32/kylinbot/srv/src/rcv_srv.c \
/home/bj/workspace/eclipse/stm32/kylinbot/srv/src/srv.c \
/home/bj/workspace/eclipse/stm32/kylinbot/srv/src/tim_srv.c \
/home/bj/workspace/eclipse/stm32/kylinbot/srv/src/tty_srv.c 

OBJS += \
./srv/src/can_srv.o \
./srv/src/rcv_srv.o \
./srv/src/srv.o \
./srv/src/tim_srv.o \
./srv/src/tty_srv.o 

C_DEPS += \
./srv/src/can_srv.d \
./srv/src/rcv_srv.d \
./srv/src/srv.d \
./srv/src/tim_srv.d \
./srv/src/tty_srv.d 


# Each subdirectory must supply rules for building sources it contributes
srv/src/can_srv.o: /home/bj/workspace/eclipse/stm32/kylinbot/srv/src/can_srv.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F4 -DSTM32F427IIHx -DSTM32 -DDEBUG -DSTM32F427X -DUSE_STDPERIPH_DRIVER -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/core" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/device" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../bsp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../srv/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../asp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../app/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../usr/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/StdPeriph_Driver/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

srv/src/rcv_srv.o: /home/bj/workspace/eclipse/stm32/kylinbot/srv/src/rcv_srv.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F4 -DSTM32F427IIHx -DSTM32 -DDEBUG -DSTM32F427X -DUSE_STDPERIPH_DRIVER -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/core" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/device" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../bsp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../srv/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../asp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../app/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../usr/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/StdPeriph_Driver/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

srv/src/srv.o: /home/bj/workspace/eclipse/stm32/kylinbot/srv/src/srv.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F4 -DSTM32F427IIHx -DSTM32 -DDEBUG -DSTM32F427X -DUSE_STDPERIPH_DRIVER -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/core" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/device" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../bsp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../srv/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../asp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../app/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../usr/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/StdPeriph_Driver/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

srv/src/tim_srv.o: /home/bj/workspace/eclipse/stm32/kylinbot/srv/src/tim_srv.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F4 -DSTM32F427IIHx -DSTM32 -DDEBUG -DSTM32F427X -DUSE_STDPERIPH_DRIVER -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/core" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/device" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../bsp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../srv/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../asp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../app/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../usr/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/StdPeriph_Driver/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

srv/src/tty_srv.o: /home/bj/workspace/eclipse/stm32/kylinbot/srv/src/tty_srv.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F4 -DSTM32F427IIHx -DSTM32 -DDEBUG -DSTM32F427X -DUSE_STDPERIPH_DRIVER -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/core" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/CMSIS/device" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../bsp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../srv/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../asp/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../app/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../usr/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl/StdPeriph_Driver/inc" -I"/home/bj/workspace/eclipse/stm32/kylinbot/prj/SW4STM32/../../stl" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


