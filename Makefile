
# toolchain
TOOLCHAIN    = arm-none-eabi-
CC           = $(TOOLCHAIN)gcc
CP           = $(TOOLCHAIN)objcopy
AS           = $(TOOLCHAIN)gcc -x assembler-with-cpp
HEX          = $(CP) -O ihex
BIN          = $(CP) -O binary -S

# define mcu, specify the target processor
MCU          = cortex-m3

# define root dir
ROOT_DIR     = .

# define include dir
INCLUDE_DIRS =

# define bin dir
BIN_DIR      = $(ROOT_DIR)/bin

# define lib dir
LIB_DIR      = $(ROOT_DIR)/stm32f10x_libraries

# define freertos dir
FREERTOS_DIR = $(ROOT_DIR)/freertos

# define user dir
USER_DIR     = $(ROOT_DIR)/user

# link file
LINK_SCRIPT  = $(ROOT_DIR)/stm32_flash.ld

# user specific
SRC       =
ASM_SRC   =
SRC      += $(USER_DIR)/main.c
SRC      += $(USER_DIR)/driver/gpio/gpio.c 
SRC      += $(USER_DIR)/driver/usart/usart.c 
SRC      += $(USER_DIR)/driver/crc/crc.c 

# user include
INCLUDE_DIRS  += $(USER_DIR)

#user include
INCLUDE_DIRS  +=$(USER_DIR)/driver
INCLUDE_DIRS  +=$(USER_DIR)/driver/gpio
INCLUDE_DIRS  +=$(USER_DIR)/driver/usart
INCLUDE_DIRS  +=$(USER_DIR)/driver/crc

# include sub makefiles
include makefile_std_lib.mk   # STM32 Standard Peripheral Library
include makefile_freertos.mk  # freertos source

INCDIR  = $(patsubst %, -I%, $(INCLUDE_DIRS))


# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJECT_NAME=$(BIN_DIR)/main

# run from Flash
DEFS	 = $(DDEFS) -DRUN_FROM_FLASH=1

OBJECTS  = $(ASM_SRC:.s=.o) $(SRC:.c=.o)

# Define optimisation level here
OPT = -O1

MC_FLAGS = -mcpu=$(MCU)

AS_FLAGS = $(MC_FLAGS) $(OPT) -g -gdwarf-2 -mthumb -mlittle-endian -Wa,-amhls=$(<:.s=.lst)
CP_FLAGS = $(MC_FLAGS) $(OPT) -g -gdwarf-2 -mthumb  -fno-short-enums -mlittle-endian -Wa,-amhls=$(<:.c=.lst) -fomit-frame-pointer -Wall -Wno-switch -fverbose-asm $(DEFS) -ffunction-sections -fdata-sections --specs=nosys.specs 
LD_FLAGS = $(MC_FLAGS) $(OPT) -g -gdwarf-2 -mthumb -mlittle-endian -nostartfiles -Xlinker --gc-sections -T$(LINK_SCRIPT) -Wl,-Map=$(PROJECT_NAME).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIB) -fno-use-cxa-atexit 



default: all
.PHONY: out all clean



#
# makefile rules
#
all: $(OBJECTS) $(PROJECT_NAME).elf  $(PROJECT_NAME).hex $(PROJECT_NAME).bin
	$(TOOLCHAIN)size $(PROJECT_NAME).elf

%.o: %.c
	@echo CC: $<
	@$(CC) -c $(CP_FLAGS) -I . $(INCDIR) $< -o $@

%.o: %.s
	@echo AS: $<
	@$(AS) -c $(AS_FLAGS) $< -o $@

%elf: $(OBJECTS)
	@$(CC) $(OBJECTS) $(LD_FLAGS) $(LIBS) -o $@

%hex: %elf
	@$(HEX) $< $@

%bin: %elf
	@$(BIN)  $< $@

flash: $(PROJECT).bin
	@st-flash write $(PROJECT).bin 0x8000000

erase:
	@st-flash erase

clean:
	@-rm -rf $(OBJECTS)
	@-rm -rf $(PROJECT_NAME).elf
	@-rm -rf $(PROJECT_NAME).map
	@-rm -rf $(PROJECT_NAME).hex
	@-rm -rf $(PROJECT_NAME).bin
	@-rm -rf $(SRC:.c=.lst)
	@-rm -rf $(ASM_SRC:.s=.lst)

