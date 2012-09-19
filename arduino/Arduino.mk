########################################################################
# Arduino 22 make file. 
#
# Adaptation (C) Donald Delmar Davis, Suspect Devices
#
# This is a dirty hack of version 0.9 26.iv.2012 of  M J Oldfield
# Arduino command line tools Makefile
#
# System part (i.e. project independent)
#
# Copyright (C) 2010,2011,2012 Martin Oldfield <m@mjo.tc>, based on
# work that is copyright Nicholas Zambetti, David A. Mellis & Hernando
# Barragan.
# 
# This file is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation; either version 2.1 of the
# License, or (at your option) any later version.
#
# Adapted from Arduino 0011 Makefile by M J Oldfield
#
# Original Arduino adaptation by mellis, eighthave, oli.keller
#                      
########################################################################
# PATHS
# I assume that unless ARDUINO_DIR is defined that the arduino core is in 
# ../../arduino/cores (it should probably be relative to this file)
# I also assume that unless the AVR_TOOLS_DIR is defined that the 
# avr-gcc toolchain is in your path. 
########################################################################
#
# cleanup (the sections around resetting the board and serial monitoring
# need to be gotten rid of.
#
########################################################################
#
# Given a normal sketch directory, all you need to do is to create
# a small Makefile which defines a few things, and then includes this one.
#
# For example:
#
#       ARDUINO_LIBS = Ethernet Ethernet/utility SPI
#       MCU    = atmega2560
#       AVRDUDE_PORT =   /dev/cu.usbmodem12a1
#
#       include /usr/local/share/Arduino.mk
#
# Hopefully these will be self-explanatory but in case they're not:
#
#    ARDUINO_LIBS - A list of any libraries used by the sketch (we
#                   assume these are in
#                   $(ARDUINO_DIR)/hardware/libraries 
#
#    AVRDUDE_PORT - The port where the Arduino can be found (only needed
#                   when uploading
#
#    MCU    -  the name of the processor
#
# Once this file has been created the typical workflow is just
#
#   $ make install
#
# All of the object files are created in the build-cli subdirectory
# All local sources should be in the current directory and can include:
#  - at most one .pde or .ino file which will be treated as C++ after
#    the standard Arduino header and footer have been affixed.
#  - any number of .c, .cpp, .s and .h files
#
# Included libraries are built in the build-cli/libs subdirectory.
#
# Besides make upload you can also
#   make             - no upload
#   make clean       - remove all our dependencies
#   make depends     - update dependencies
#   make install     - connect to the Arduino's serial port
#
########################################################################
########################################################################
#
# ARDUINO WITH ISP
#
# You need to specify some details of your ISP programmer and might
# also need to specify the fuse values:
#
#     AVRDUDE_PROGRAMMER	   = -c stk500v2
#     AVRDUDE_PORT     = /dev/ttyACM0
#
# You might also need to set the fuse bits, but typically they'll be
#     
#     ISP_LOCK_FUSE_PRE  = 0x3f
#     ISP_LOCK_FUSE_POST = 0xcf
#     ISP_HIGH_FUSE      = 0xdf
#     ISP_LOW_FUSE       = 0xff
#     ISP_EXT_FUSE       = 0x01
#
# I think the fuses here are fine for uploading to the ATmega168
# without bootloader.
# 
# To actually do this upload use the ispload target:
#
#    make ispload
#
#
########################################################################

########################################################################
# 
# Default TARGET to cwd (ex Daniele Vergini)
ifndef TARGET
TARGET  = $(notdir $(CURDIR))
endif

########################################################################
#
# Arduino version number
ifndef ARDUINO_VERSION
ARDUINO_VERSION = 0022
endif

########################################################################
# figure out what system we are on.
# Uname=Darwin
# Uname=Linux
# (defaults to Windows)
# ******************* PATHS ARE HARDCODED *********************
# the firstword works on the macintosh if there is only one
# you can use an external script to guess.
# 

#$

UNAME := $(shell uname -s)

ifeq ($(UNAME),Darwin)
    ifndef AVRDUDE_PORT
        AVRDUDE_PORT=$(firstword $(wildcard /dev/tty.usbmodem*))
    endif
else 
ifeq ($(UNAME),Linux)
    ifndef AVRDUDE_PORT
        AVRDUDE_PORT=/dev/ttyACM0
    endif
else
	UNAME=Windows
    ifndef AVRDUDE_PORT
        AVRDUDE_PORT=COM8:
    endif
endif
endif


########################################################################
# Arduino and system paths taylor to your needs..
#
ifdef ARDUINO_DIR

ifndef AVR_TOOLS_DIR
AVR_TOOLS_DIR     = $(ARDUINO_DIR)/hardware/tools/avr
# The avrdude bundled with Arduino can't find it's config
AVRDUDE_CONF	  = $(AVR_TOOLS_DIR)/etc/avrdude.conf
endif

ifndef AVR_TOOLS_PATH
AVR_TOOLS_PATH    = $(AVR_TOOLS_DIR)/bin
endif

ARDUINO_LIB_PATH  = $(ARDUINO_DIR)/libraries
ARDUINO_CORE_PATH = $(ARDUINO_DIR)/hardware/arduino/cores/arduino
ARDUINO_VAR_PATH  = $(ARDUINO_DIR)/hardware/arduino/variants

else

ARDUINO_LIB_PATH  = ../../arduino/libraries
ARDUINO_CORE_PATH = ../../arduino/cores/arduino
ifeq ($(UNAME),Windows)
    AVRDUDE_CONF = ../../arduino/avrdude.conf
endif 
ARDUINO_VAR_PATH  = .

#echo $(error "ARDUINO_DIR is not defined")

endif



########################################################################
# Miscellanea
#
ifndef ARDUINO_SKETCHBOOK
ARDUINO_SKETCHBOOK = $(HOME)/sketchbook
endif

ifndef USER_LIB_PATH
USER_LIB_PATH = ../../libraries
endif

# Which variant ? This affects the include path for arduino 1.0 
ifndef VARIANT
VARIANT = mega2560
endif

# processor stuff
ifndef MCU
MCU   = atmega2560
endif

ifndef F_CPU
F_CPU = 16000000
endif

# normal programming info
ifndef AVRDUDE_PROGRAMMER
AVRDUDE_PROGRAMMER = stk500v2
endif

ifndef AVRDUDE_BAUDRATE
AVRDUDE_BAUDRATE = 115200
endif

# fuses if you're using e.g. ISP
ifndef ISP_LOCK_FUSE_PRE
ISP_LOCK_FUSE_PRE  = 0x3F
endif

ifndef ISP_LOCK_FUSE_POST
ISP_LOCK_FUSE_POST = 0x0F
endif

ifndef ISP_HIGH_FUSE
ISP_HIGH_FUSE      = 0xD8
endif

ifndef ISP_LOW_FUSE
ISP_LOW_FUSE       = 0xFF
endif

ifndef ISP_EXT_FUSE
ISP_EXT_FUSE       =0xFD
endif

# Everything gets built in here
OBJDIR  	  = build-cli

########################################################################
# Local sources
#
LOCAL_C_SRCS    = $(wildcard *.c)
LOCAL_CPP_SRCS  = $(wildcard *.cpp)
LOCAL_CC_SRCS   = $(wildcard *.cc)
LOCAL_PDE_SRCS  = $(wildcard *.pde)
LOCAL_INO_SRCS  = $(wildcard *.ino)
LOCAL_AS_SRCS   = $(wildcard *.S)
LOCAL_OBJ_FILES = $(LOCAL_C_SRCS:.c=.o)   $(LOCAL_CPP_SRCS:.cpp=.o) \
		$(LOCAL_CC_SRCS:.cc=.o)   $(LOCAL_PDE_SRCS:.pde=.o) \
		$(LOCAL_INO_SRCS:.ino=.o) $(LOCAL_AS_SRCS:.S=.o)
LOCAL_OBJS      = $(patsubst %,$(OBJDIR)/%,$(LOCAL_OBJ_FILES))

# Dependency files
DEPS            = $(LOCAL_OBJS:.o=.d)

# core sources
ifeq ($(strip $(NO_CORE)),)
ifdef ARDUINO_CORE_PATH
CORE_C_SRCS     = $(wildcard $(ARDUINO_CORE_PATH)/*.c)
CORE_CPP_SRCS   = $(wildcard $(ARDUINO_CORE_PATH)/*.cpp)

ifneq ($(strip $(NO_CORE_MAIN_CPP)),)
CORE_CPP_SRCS := $(filter-out %main.cpp, $(CORE_CPP_SRCS))
endif

CORE_OBJ_FILES  = $(CORE_C_SRCS:.c=.o) $(CORE_CPP_SRCS:.cpp=.o)
CORE_OBJS       = $(patsubst $(ARDUINO_CORE_PATH)/%,  \
			$(OBJDIR)/%,$(CORE_OBJ_FILES))
endif
endif


########################################################################
# Rules for making stuff
#

# The name of the main targets
TARGET_HEX = $(OBJDIR)/$(TARGET).hex
TARGET_ELF = $(OBJDIR)/$(TARGET).elf
TARGETS    = $(OBJDIR)/$(TARGET).*
CORE_LIB   = $(OBJDIR)/libcore.a

# A list of dependencies
DEP_FILE   = $(OBJDIR)/depends.mk

# Names of executables
#
ifdef AVR_TOOLS_PATH
CC      = $(AVR_TOOLS_PATH)/avr-gcc
CXX     = $(AVR_TOOLS_PATH)/avr-g++
OBJCOPY = $(AVR_TOOLS_PATH)/avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)/avr-objdump
AR      = $(AVR_TOOLS_PATH)/avr-ar
SIZE    = $(AVR_TOOLS_PATH)/avr-size
NM      = $(AVR_TOOLS_PATH)/avr-nm
AVRDUDE = $(AVR_TOOLS_PATH)/avrdude
else
CC      = avr-gcc
CXX     = avr-g++
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
AR      = avr-ar
SIZE    = avr-size
NM      = avr-nm
AVRDUDE = avrdude
endif

REMOVE  = rm -f
MV      = mv -f
CAT     = cat
ECHO    = echo

# General arguments
SYS_LIBS      = $(patsubst %,$(ARDUINO_LIB_PATH)/%,$(ARDUINO_LIBS))
EXTRA_LIBS     = $(patsubst %,$(USER_LIB_PATH)/%,$(USER_LIBS))
SYS_INCLUDES  = $(patsubst %,-I%,$(SYS_LIBS))
USER_INCLUDES = $(patsubst %,-I%,$(EXTRA_LIBS))
LIB_C_SRCS    = $(wildcard $(patsubst %,%/*.c,$(SYS_LIBS)))
LIB_CPP_SRCS  = $(wildcard $(patsubst %,%/*.cpp,$(SYS_LIBS)))
USER_LIB_CPP_SRCS   = $(wildcard $(patsubst %,%/*.cpp,$(EXTRA_LIBS)))
USER_LIB_C_SRCS     = $(wildcard $(patsubst %,%/*.c,$(EXTRA_LIBS)))
LIB_OBJS      = $(patsubst $(ARDUINO_LIB_PATH)/%.c,$(OBJDIR)/libs/%.o,$(LIB_C_SRCS)) \
		$(patsubst $(ARDUINO_LIB_PATH)/%.cpp,$(OBJDIR)/libs/%.o,$(LIB_CPP_SRCS))
USER_LIB_OBJS = $(patsubst $(USER_LIB_PATH)/%.cpp,$(OBJDIR)/libs/%.o,$(USER_LIB_CPP_SRCS)) \
		$(patsubst $(USER_LIB_PATH)/%.c,$(OBJDIR)/libs/%.o,$(USER_LIB_C_SRCS))

CPPFLAGS      = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -DARDUINO=$(ARDUINO_VERSION) \
			-I. -I$(ARDUINO_CORE_PATH) -I$(ARDUINO_VAR_PATH)/$(VARIANT) \
			$(SYS_INCLUDES) $(USER_INCLUDES) -g -Os -w -Wall \
			-ffunction-sections -fdata-sections
CFLAGS        = -std=gnu99
CXXFLAGS      = -fno-exceptions
ASFLAGS       = -mmcu=$(MCU) -I. -x assembler-with-cpp 
LDFLAGS       = -mmcu=$(MCU) -Wl,--gc-sections -Os $(USER_LDFLAGS)

# Expand and pick the first port
# ARD_PORT      = $(firstword $(wildcard $(AVRDUDE_PORT)))

# Implicit rules for building everything (needed to get everything in
# the right directory)
#
# Rather than mess around with VPATH there are quasi-duplicate rules
# here for building e.g. a system C++ file and a local C++
# file. Besides making things simpler now, this would also make it
# easy to change the build options in future

# library sources
$(OBJDIR)/libs/%.o: $(ARDUINO_LIB_PATH)/%.c
	mkdir -p $(dir $@)
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/libs/%.o: $(ARDUINO_LIB_PATH)/%.cpp
	mkdir -p $(dir $@)
	$(CC) -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/libs/%.o: $(USER_LIB_PATH)/%.cpp
	mkdir -p $(dir $@)
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/libs/%.o: $(USER_LIB_PATH)/%.c
	mkdir -p $(dir $@)
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $< -o $@

# normal local sources
# .o rules are for objects, .d for dependency tracking
# there seems to be an awful lot of duplication here!!!
$(OBJDIR)/%.o: %.c
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: %.cc
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/%.o: %.cpp
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/%.o: %.S
	$(CC) -c $(CPPFLAGS) $(ASFLAGS) $< -o $@

$(OBJDIR)/%.o: %.s
	$(CC) -c $(CPPFLAGS) $(ASFLAGS) $< -o $@

$(OBJDIR)/%.d: %.c
	$(CC) -MM $(CPPFLAGS) $(CFLAGS) $< -MF $@ -MT $(@:.d=.o)

$(OBJDIR)/%.d: %.cc
	$(CXX) -MM $(CPPFLAGS) $(CXXFLAGS) $< -MF $@ -MT $(@:.d=.o)

$(OBJDIR)/%.d: %.cpp
	$(CXX) -MM $(CPPFLAGS) $(CXXFLAGS) $< -MF $@ -MT $(@:.d=.o)

$(OBJDIR)/%.d: %.S
	$(CC) -MM $(CPPFLAGS) $(ASFLAGS) $< -MF $@ -MT $(@:.d=.o)

$(OBJDIR)/%.d: %.s
	$(CC) -MM $(CPPFLAGS) $(ASFLAGS) $< -MF $@ -MT $(@:.d=.o)

# the pde -> cpp -> o file
$(OBJDIR)/%.cpp: %.pde
	$(ECHO) '#include "WProgram.h"' > $@
	$(CAT)  $< >> $@

# the ino -> cpp -> o file
$(OBJDIR)/%.cpp: %.ino
	$(ECHO) '#include <Arduino.h>' > $@
	$(CAT)  $< >> $@

$(OBJDIR)/%.o: $(OBJDIR)/%.cpp
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/%.d: $(OBJDIR)/%.cpp
	$(CXX) -MM $(CPPFLAGS) $(CXXFLAGS) $< -MF $@ -MT $(@:.d=.o)

# core files
$(OBJDIR)/%.o: $(ARDUINO_CORE_PATH)/%.c
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: $(ARDUINO_CORE_PATH)/%.cpp
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

# various object conversions
$(OBJDIR)/%.hex: $(OBJDIR)/%.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(OBJDIR)/%.eep: $(OBJDIR)/%.elf
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
		--change-section-lma .eeprom=0 -O ihex $< $@

$(OBJDIR)/%.lss: $(OBJDIR)/%.elf
	$(OBJDUMP) -h -S $< > $@

$(OBJDIR)/%.sym: $(OBJDIR)/%.elf
	$(NM) -n $< > $@

########################################################################
#
# Avrdude
#

AVRDUDE_COM_OPTS =  -D -p $(MCU)
ifdef AVRDUDE_CONF
AVRDUDE_COM_OPTS += -C $(AVRDUDE_CONF)
endif

AVRDUDE_ARD_OPTS = -c $(AVRDUDE_PROGRAMMER) -b $(AVRDUDE_BAUDRATE) -P $(AVRDUDE_PORT)

ifndef AVRDUDE_PROGRAMMER
AVRDUDE_PROGRAMMER	   = -c stk500v2
endif

AVRDUDE_ISP_OPTS = -P $(AVRDUDE_PORT) $(AVRDUDE_PROGRAMMER)


########################################################################
#
# Explicit targets start here
#

all: 		$(OBJDIR) $(TARGET_HEX)

$(OBJDIR):
		mkdir $(OBJDIR)

$(TARGET_ELF): 	$(LOCAL_OBJS) $(CORE_LIB) $(OTHER_OBJS)
		$(CC) $(LDFLAGS) -o $@ $(LOCAL_OBJS) $(CORE_LIB) $(OTHER_OBJS) -lc -lm

$(CORE_LIB):	$(CORE_OBJS) $(LIB_OBJS) $(USER_LIB_OBJS)
		$(AR) rcs $@ $(CORE_OBJS) $(LIB_OBJS) $(USER_LIB_OBJS)

$(DEP_FILE):	$(OBJDIR) $(DEPS)
		cat $(DEPS) > $(DEP_FILE)

install: upload

program: upload

upload:	$(TARGET_HEX)
		$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ARD_OPTS) \
			-U flash:w:$(TARGET_HEX):i

ispload:	$(TARGET_HEX)
		$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) -e \
			-U lock:w:$(ISP_LOCK_FUSE_PRE):m \
			-U hfuse:w:$(ISP_HIGH_FUSE):m \
			-U lfuse:w:$(ISP_LOW_FUSE):m \
			-U efuse:w:$(ISP_EXT_FUSE):m
		$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) -D \
			-U flash:w:$(TARGET_HEX):i
		$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) \
			-U lock:w:$(ISP_LOCK_FUSE_POST):m

clean:
		$(REMOVE) $(LOCAL_OBJS) $(CORE_OBJS) $(LIB_OBJS) $(CORE_LIB) $(TARGETS) $(DEP_FILE) $(DEPS) $(USER_LIB_OBJS)

depends:	$(DEPS)
		cat $(DEPS) > $(DEP_FILE)

size:		$(OBJDIR) $(TARGET_HEX)
		$(SIZE) $(TARGET_HEX)


.PHONY:	all clean depends upload reset size  monitor

include $(DEP_FILE)
