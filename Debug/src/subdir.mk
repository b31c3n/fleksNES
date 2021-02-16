################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/16_bit.c \
../src/addr_modes.c \
../src/api.c \
../src/apu.c \
../src/bus.c \
../src/clock.c \
../src/colors.c \
../src/cpu.c \
../src/display.c \
../src/helper_funcs.c \
../src/instruction.c \
../src/instruction_tbl.c \
../src/log.c \
../src/main.c \
../src/mapper.c \
../src/mapper000.c \
../src/mapper001.c \
../src/ppu.c \
../src/ppu_comm.c \
../src/ram.c \
../src/state.c \
../src/tui.c 

OBJS += \
./src/16_bit.o \
./src/addr_modes.o \
./src/api.o \
./src/apu.o \
./src/bus.o \
./src/clock.o \
./src/colors.o \
./src/cpu.o \
./src/display.o \
./src/helper_funcs.o \
./src/instruction.o \
./src/instruction_tbl.o \
./src/log.o \
./src/main.o \
./src/mapper.o \
./src/mapper000.o \
./src/mapper001.o \
./src/ppu.o \
./src/ppu_comm.o \
./src/ram.o \
./src/state.o \
./src/tui.o 

C_DEPS += \
./src/16_bit.d \
./src/addr_modes.d \
./src/api.d \
./src/apu.d \
./src/bus.d \
./src/clock.d \
./src/colors.d \
./src/cpu.d \
./src/display.d \
./src/helper_funcs.d \
./src/instruction.d \
./src/instruction_tbl.d \
./src/log.d \
./src/main.d \
./src/mapper.d \
./src/mapper000.d \
./src/mapper001.d \
./src/ppu.d \
./src/ppu_comm.d \
./src/ram.d \
./src/state.d \
./src/tui.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -IGl -O0 -g -w -c -fmessage-length=0 -fopenmp -pthread -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


