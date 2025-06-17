################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
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
SECONDARY_FLASH += \
UAVCAN_Controller.hex \

SECONDARY_LIST += \
UAVCAN_Controller.lst \

SECONDARY_SIZE += \
UAVCAN_Controller.siz \


# All Target
all: UAVCAN_Controller.elf secondary-outputs

# Tool invocations
UAVCAN_Controller.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -T "C:\Users\roman\Desktop\UAVCAN_Controller\UAVCAN_Controller\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"UAVCAN_Controller.map" --specs=nano.specs --specs=nosys.specs -o "UAVCAN_Controller.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
UAVCAN_Controller.hex: UAVCAN_Controller.elf
	@	riscv-none-embed-objcopy -O ihex "UAVCAN_Controller.elf"  "UAVCAN_Controller.hex"
	@	@
UAVCAN_Controller.lst: UAVCAN_Controller.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble -M xw "UAVCAN_Controller.elf" > "UAVCAN_Controller.lst"
	@	@
UAVCAN_Controller.siz: UAVCAN_Controller.elf
	@	riscv-none-embed-size --format=berkeley "UAVCAN_Controller.elf"
	@	@
# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) UAVCAN_Controller.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
