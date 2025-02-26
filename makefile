# VEXcode Makefile Modified

# show compiler output
VERBOSE = 0

# Include toolchain options
include vex/mkenv.mk

# Location of the project source C and C++ files
SRC_C = $(shell find src -type f \( -name "*.c" -o -name "*.cpp" \))
SRC_C += $(shell find include/lvgl/src -type f \( -name "*.c" -o -name "*.cpp" \))

# Generate object file list
OBJ = $(patsubst %,$(BUILD)/%.o,$(basename $(SRC_C)))

# Location of include files
SRC_H = $(shell find include -type f -name "*.h")
SRC_H += lv_conf.h

# Additional dependencies
SRC_A = makefile

# Project header file locations
INC_F = include . lvgl lvgl/src

# Headers needed to use the library
LV_SRC_H = $(shell find include/lvgl -type f -name "*.h")
LV_DST_H = $(patsubst %,$(BUILD)/include/%,$(LV_SRC_H))
LV_DST_H += $(BUILD)/include/lv_conf.h $(BUILD)/include/v5lvgl.h

# Ensure headers are copied to build folder
$(BUILD)/include/%: %
	$(Q)$(MKDIR)
	$(Q)$(call copyfile,$^, $@)

# Search paths for header files
vpath %.h lvgl/ include/

# Override default library name
PROJECTLIB = libv5lvgl

# Build targets
all: $(BUILD)/$(PROJECT).bin

# Copy LVGL header files
.PHONY: inc
inc: $(LV_DST_H)
	$(ECHO) "Copy headers to build folder"

# Include build rules
include vex/mkrules.mk
