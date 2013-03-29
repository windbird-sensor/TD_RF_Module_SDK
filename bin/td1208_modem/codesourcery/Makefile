####################################################################
# @file
# @brief Makefile for TD1202 RF module.
# @author Telecom Design S.A.
# @version 1.0.0
####################################################################
# @section License
# <b>(C) Copyright 2012 Telecom Design S.A., http://www.telecom-design.com</b>
####################################################################
#
# This source code is the property of Telecom Design S.A.
#
# This copyright notice may not be removed from the source code nor changed.
#
####################################################################

# Ignore builtin rules
.SUFFIXES:

# Do not create real files for these targets
.PHONY: all Debug Release clean

####################################################################
# Definitions                                                      #
####################################################################

# Device part
DEVICE = EFM32TG210F32

# Project name
PROJECTNAME = td1202

# Energy Micro EFM32 distribution root directory
EFM32_DIR = ../..

# CMSIS directory
CMSIS_DIR = $(EFM32_DIR)/CMSIS

# Energy Micro EFM32 library directory
EFM32LIB_DIR = $(EFM32_DIR)/efm32lib

# Project root directory
PROJECT_DIR = ../../ 

# Uncomment following line for verbose build
V = 1

####################################################################
# Files                                                            #
####################################################################

# C source files
C_SRC +=  \
$(CMSIS_DIR)/CM3/CoreSupport/core_cm3.c \
$(CMSIS_DIR)/CM3/DeviceSupport/EnergyMicro/EFM32/system_efm32.c \
$(wildcard $(PROJECT_DIR)/*.c) \
$(EFM32LIB_DIR)/src/efm32_assert.c \
$(EFM32LIB_DIR)/src/efm32_system.c \
$(EFM32LIB_DIR)/src/efm32_cmu.c \
$(EFM32LIB_DIR)/src/efm32_emu.c \
$(EFM32LIB_DIR)/src/efm32_gpio.c \
$(EFM32LIB_DIR)/src/efm32_usart.c \
$(EFM32LIB_DIR)/src/efm32_timer.c \
$(EFM32LIB_DIR)/src/efm32_rtc.c \
$(EFM32LIB_DIR)/src/efm32_leuart.c \
$(EFM32LIB_DIR)/src/efm32_adc.c \
../../libtd1202/src/td_flash.c \
../../libtd1202/src/td_cmu.c \
../../libtd1202/src/td_gpio.c \
../../libtd1202/src/td_uart.c \
../../libtd1202/src/td_printf.c\
../../libtd1202/src/td_spi.c\
../../libtd1202/src/td_timer.c\
../../libtd1202/src/td_rtc.c\
../../libtd1202/src/td_rf.c\
../../libtd1202/src/td_rf_Si4461.c\
../../libtd1202/src/td_loader.c \
../src/at_parse.c \
../src/at_sigfox.c \
../src/at_radio.c \
../src/at_local.c \
../src/gpio_test.c \
../src/at_mfg_test.c \
../src/rf_cert_test.c \
../src/rf_cem_test.c \
../src/main.c \
../src/td_sigfox.c \
#../../libtd1202/src/td_sigfox.c \
#../src/TransceiverSigfox.c \

#../src/rf_covea.c \
../src/rf_mfg_test.c \
# Assembler source files
S_SRC +=  \
$(CMSIS_DIR)/CM3/DeviceSupport/EnergyMicro/EFM32/startup/cs3/startup_efm32tg.s

####################################################################
# Definitions of toolchain                                         #
# You might need to do changes to match your system setup          #
####################################################################

# Change path to CodeSourcery tools according to your system configuration
WINDOWSCS = CodeSourcery/Sourcery G++ Lite
LINUXCS   = /usr/local/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI
GCCVERSION = $(shell $(CC) -dumpversion)
EACOM = $(HOME)/efm32_starter_kit/energymicro/eACommander.sh

ifeq ($(MAKECMDGOALS),clean)
BUILD_DIR = */build
EXE_DIR = */exe
LST_DIR = */lst
else
ifeq ($(MAKECMDGOALS),flash)
BUILD_DIR = */build
EXE_DIR = */exe
LST_DIR = */lst
else
BUILD_DIR = $(MAKECMDGOALS)/build
EXE_DIR = $(MAKECMDGOALS)/exe
LST_DIR = $(MAKECMDGOALS)/lst
endif
endif

# Try to autodetect platform
ifeq ($(ComSpec),)
  ifeq ($(COMSPEC),)
    # Assume we are making on a linux platform
    TOOL_DIR = $(LINUXCS)
    RM = rm -rf
  else
    TOOL_DIR = $(PROGRAMFILES)/$(WINDOWSCS)
    RM = "$(TOOL_DIR)/bin/cs-rm" -rf
    QUOTE ="
  endif
else
  TOOL_DIR = $(ProgramFiles)/$(WINDOWSCS)
  RM = "$(TOOL_DIR)/bin/cs-rm" -rf
  QUOTE ="
endif

# Build verbosity
ifeq ($(V),)
  Q = @
else
  Q =
endif

MKDIR	= mkdir -p
CC      = $(QUOTE)$(TOOL_DIR)/bin/arm-none-eabi-gcc$(QUOTE)
LD      = $(QUOTE)$(TOOL_DIR)/bin/arm-none-eabi-ld$(QUOTE)
AR      = $(QUOTE)$(TOOL_DIR)/bin/arm-none-eabi-ar$(QUOTE)
OBJCOPY = $(QUOTE)$(TOOL_DIR)/bin/arm-none-eabi-objcopy$(QUOTE)
DUMP    = $(QUOTE)$(TOOL_DIR)/bin/arm-none-eabi-objdump$(QUOTE) --disassemble
SIZE    = $(QUOTE)$(TOOL_DIR)/bin/arm-none-eabi-size$(QUOTE)
                                                      
####################################################################
# Flags                                                            #
####################################################################

# Symbol definitions
DEFINES += -D$(DEVICE)

# Dependencies
# -MMD : Don't generate dependencies on system header files.
# -MP  : Add phony targets, useful when a h-file is removed from a project.
# -MF  : Specify a file to write the dependencies to.
DEPFLAGS = -MMD -MP -MF $(@:.o=.d)

# Include paths
INCLUDEPATHS += \
-I$(PROJECT_DIR) \
-I$(CMSIS_DIR)/CM3/CoreSupport \
-I$(CMSIS_DIR)/CM3/DeviceSupport/EnergyMicro/EFM32 \
-I$(EFM32LIB_DIR)/inc

INCLUDEPATHS += \
-I.. \
-I../../CMSIS/CM3/CoreSupport \
-I../../CMSIS/CM3/DeviceSupport/EnergyMicro/EFM32 \
-I../../efm32lib/inc \
-I../../libtd1202/inc/ \
-I../../td1202/inc

# C compiler flags
# Add -Wa,-ahld=$(LST_DIR)/$(@F:.o=.lst) to CFLAGS to produce assembly list files
CFLAGS += \
-Wall \
-fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections \
-mfix-cortex-m3-ldrd -fomit-frame-pointer -fno-exceptions \
-mcpu=cortex-m3 -mthumb \
$(DEPFLAGS)

# Assembler flags
ASMFLAGS += -Ttext 0x0

# Library paths
LIBPATHS += \
-L"$(EFM32LIB_DIR)/codesourcery/$(MAKECMDGOALS)/lib" \
-L"$(TOOL_DIR)/arm-none-eabi/lib/thumb2" \
-L"$(TOOL_DIR)/lib/gcc/arm-none-eabi/$(GCCVERSION)/thumb2"

# Linker flags
LDFLAGS += \
-Xlinker --gc-sections \
-Xlinker -Map=$(LST_DIR)/$(PROJECTNAME).map -mcpu=cortex-m3 -mthumb \
-mcpu=cortex-m3 -mthumb \
-T$(CMSIS_DIR)/CM3/DeviceSupport/EnergyMicro/EFM32/startup/cs3/efm32tg.ld

# Libraries
LIBS += -lc -lcs3 -lcs3unhosted

#LIBS += -lefm32

####################################################################
# Rules                                                            #
####################################################################

# Remove directories from file names
C_FILES = $(notdir $(C_SRC) )
S_FILES = $(notdir $(S_SRC) )

# Make list of source paths, sort also removes duplicates
C_PATHS = $(sort $(dir $(C_SRC) ) )
S_PATHS = $(sort $(dir $(S_SRC) ) )

# Put all intermediate generated files into build directory 
C_OBJS = $(addprefix $(BUILD_DIR)/, $(C_FILES:.c=.o))
S_OBJS = $(addprefix $(BUILD_DIR)/, $(S_FILES:.s=.o))
C_DEPS = $(addprefix $(BUILD_DIR)/, $(C_FILES:.c=.d))

# Specify the search paths for C and assembly files
vpath %.c $(C_PATHS)
vpath %.s $(S_PATHS)

table: clean
		octave --silent --eval "PSK" > ../inc/pa_table.h
		
# Default build is debug build
all:    Release

# Debug has no optimization and generate debug sybols
Debug:    CFLAGS += -DDEBUG -DDEBUG_EFM -O0 -g3
Debug:    $(BUILD_DIR) $(LST_DIR) $(EXE_DIR) $(EXE_DIR)/$(PROJECTNAME).bin

# Release has optimization for size and no debug symbols
Release:  CFLAGS += -DNDEBUG -O3
Release:  $(BUILD_DIR) $(LST_DIR) $(EXE_DIR) $(EXE_DIR)/$(PROJECTNAME).bin

flash:
	$(EACOM) -m out -f $(EXE_DIR)/$(PROJECTNAME).bin -v
run:
	$(EACOM) -r
	#gtkterm -s 9600 -p /dev/ttyUSB1

# Create directories
$(BUILD_DIR):
	@echo "MKDIR $@"
	$(Q)${MKDIR} $(BUILD_DIR)

$(EXE_DIR):
	@echo "MKDIR $@"
	$(Q)${MKDIR} $(EXE_DIR)

$(LST_DIR):
	@echo "MKDIR $@"
	$(Q)${MKDIR} $(LST_DIR)

# Create objects from C source files
$(BUILD_DIR)/%.o: %.c
	@echo "CC $(notdir $<)"
	$(Q)$(CC) $(DEFINES) $(INCLUDEPATHS) $(CFLAGS) -c -o $@ $<

# Create objects from assembly files
$(BUILD_DIR)/%.o: %.s
	@echo "AS $(notdir $<)"
	$(Q)$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

# Link executable
$(EXE_DIR)/$(PROJECTNAME).out: $(C_OBJS) $(S_OBJS)
	@echo "LD $(notdir $@)"
	$(Q)$(CC) $(LDFLAGS) $(LIBPATHS) $(C_OBJS) $(S_OBJS) $(LIBS) -o $(EXE_DIR)/$(PROJECTNAME).out

# Create binary file from executable fime
$(EXE_DIR)/$(PROJECTNAME).bin: $(EXE_DIR)/$(PROJECTNAME).out
	@echo "OBJCOPY $(notdir $@)"
	$(Q)$(OBJCOPY) -O binary $(EXE_DIR)/$(PROJECTNAME).out $(EXE_DIR)/$(PROJECTNAME).bin
# Uncomment next line to produce assembly listing of entire program
#	$(Q)$(DUMP) $(EXE_DIR)/$(PROJECTNAME).out>$(LST_DIR)/$(PROJECTNAME)out.lst
	@echo "SIZE $(notdir $<)"
	$(Q)${SIZE} $(EXE_DIR)/$(PROJECTNAME).out

# Clean the project
clean:
	$(Q)$(RM) $(BUILD_DIR) $(LST_DIR) $(EXE_DIR) Debug Release
	rm ../../libtd1202/src/td_sigfox.c
# delete this file cos of bug in makefile: take this file in libtd1202 instead of td1202/td_sigfox.c

# Include auto-generated dependency files (explicit rules)
ifneq (clean,$(findstring clean, $(MAKECMDGOALS)))
-include $(C_DEPS)
endif
