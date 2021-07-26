################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SPI_HAL_NRF_Data_Read_Wrtie.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SPI_HAL_NRF_Data_Read_Wrtie.list \

OBJCOPY_BIN += \
SPI_HAL_NRF_Data_Read_Wrtie.bin \


# All Target
all: SPI_HAL_NRF_Data_Read_Wrtie.elf secondary-outputs

# Tool invocations
SPI_HAL_NRF_Data_Read_Wrtie.elf: $(OBJS) $(USER_OBJS) D:\CubeIDE_Project\SPI_HAL_NRF_Data_Read_Wrtie\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "SPI_HAL_NRF_Data_Read_Wrtie.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\CubeIDE_Project\SPI_HAL_NRF_Data_Read_Wrtie\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SPI_HAL_NRF_Data_Read_Wrtie.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SPI_HAL_NRF_Data_Read_Wrtie.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SPI_HAL_NRF_Data_Read_Wrtie.list"
	@echo 'Finished building: $@'
	@echo ' '

SPI_HAL_NRF_Data_Read_Wrtie.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SPI_HAL_NRF_Data_Read_Wrtie.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
