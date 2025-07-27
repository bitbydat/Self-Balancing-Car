GCC_DIR := C:/GNU_Arm_Embedded_Toolchain/2021.10/
CC=$(GCC_DIR)/bin/arm-none-eabi-gcc
# Linker file
LD_FILE := linker/stm32f10x_linker.ld

MODULE := arm
# Include files
INC_DIRS += inc
INC_FILES := $(foreach INC_DIRS,$(INC_DIRS),$(wildcard $(INC_DIRS)/*))
# Source files
SRC_DIRS += src \
			startup
SRC_FILES:= $(foreach SRC_DIRS,$(SRC_DIRS),$(wildcard $(SRC_DIRS)/*))
# Output files
PATH_OUTPUT := output
OBJ_FILES = $(notdir $(SRC_FILES))
OBJ_FILES := $(subst .c,.o,$(OBJ_FILES))
OBJ_PATHS := $(foreach OBJ_FILES,$(OBJ_FILES),$(PATH_OUTPUT)/$(OBJ_FILES))

JLINK_SCRIPT := tools/jlink/test.jlink_script

CHIP=cortex-m3

CCFLAGS= -c -mcpu=$(CHIP) -mthumb -mfloat-abi=soft -std=gnu11 -O0 -g -I$(INC_DIRS)
LDFLAGS= -mcpu=$(CHIP) -mthumb -mfloat-abi=soft --specs=nosys.specs -nostartfiles -T $(LD_FILE) -Wl,-Map=$(PATH_OUTPUT)/$(MODULE).map

vpath %.c $(SRC_DIRS)

build: $(MODULE).hex

%.o: %.c
	$(CC) $(CCFLAGS) -o $(PATH_OUTPUT)/$@ $^

$(MODULE).elf: $(OBJ_FILES)
	$(CC) $(LDFLAGS) $(OBJ_PATHS) -lm -lgcc -o $(PATH_OUTPUT)/$@
	
$(MODULE).hex: $(MODULE).elf
	$(GCC_DIR)/arm-none-eabi/bin/objcopy.exe -O ihex $(PATH_OUTPUT)/$^ $(PATH_OUTPUT)/$@

.PHONY: runst
runst:
	@ST-LINK_CLI.exe -Q -ME	-P output/$(MODULE).hex 0x08000000 -Halt

.PHONY: run
run:
	@JLink.exe -ExitOnError -CommanderScript $(JLINK_SCRIPT)

print-%:
	@echo $($(subst print-,,$@))

.PHONY: clean
clean: 
	rm -rf ./$(PATH_OUTPUT)/*  