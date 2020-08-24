################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/16_bit.c \
../src/addr_modes.c \
../src/apu.c \
../src/bus.c \
../src/clock.c \
../src/colors.c \
../src/cpu.c \
../src/display.c \
../src/dma.c \
../src/helper_funcs.c \
../src/instruction.c \
../src/instruction_tbl.c \
../src/log.c \
../src/main.c \
../src/mapper.c \
../src/peripheral.c \
../src/ppu.c \
../src/ram.c \
../src/tui.c 

OBJS += \
./src/16_bit.o \
./src/addr_modes.o \
./src/apu.o \
./src/bus.o \
./src/clock.o \
./src/colors.o \
./src/cpu.o \
./src/display.o \
./src/dma.o \
./src/helper_funcs.o \
./src/instruction.o \
./src/instruction_tbl.o \
./src/log.o \
./src/main.o \
./src/mapper.o \
./src/peripheral.o \
./src/ppu.o \
./src/ram.o \
./src/tui.o 

C_DEPS += \
./src/16_bit.d \
./src/addr_modes.d \
./src/apu.d \
./src/bus.d \
./src/clock.d \
./src/colors.d \
./src/cpu.d \
./src/display.d \
./src/dma.d \
./src/helper_funcs.d \
./src/instruction.d \
./src/instruction_tbl.d \
./src/log.d \
./src/main.d \
./src/mapper.d \
./src/peripheral.d \
./src/ppu.d \
./src/ram.d \
./src/tui.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -std=c11 -IGl -O0 -g3 -w -c -fmessage-length=0 -fopenmp -pthread -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


