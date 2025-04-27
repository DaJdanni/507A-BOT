# VEXcode Makefile Modified
# Show compiler output
VERBOSE = 0

# Include toolchain options
include vex/mkenv.mk

# Detect OS and define copy and mkdir commands
ifeq ($(OS),Windows_NT)
    COPY = copy /Y
    DELIM = \
else
    COPY = cp -f
    DELIM = /
endif

# Define helper to copy files
copyfile = $(COPY) $1 $2

# Recursive wildcard function for portability
rwildcard = $(foreach d,$(wildcard $1/*),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d))

# Location of the project source C and C++ files
SRC_C = $(call rwildcard,src,*.c) $(call rwildcard,src,*.cpp) $(call rwildcard,include/lvgl/src,*.c) $(call rwildcard,include/lvgl/src,*.cpp)

# Generate object file list
OBJ = $(patsubst %,$(BUILD)/%.o,$(basename $(SRC_C)))

# Location of include files
SRC_H = $(call rwildcard,include,*.h)
SRC_H += lv_conf.h

# Additional dependencies
SRC_A = makefile

# Project header file locations
INC_F = include . lvgl lvgl/src

# Headers needed to use the library
LV_SRC_H = $(call rwildcard,include/lvgl,*.h)
LV_DST_H = $(patsubst %,$(BUILD)/include/%,$(LV_SRC_H))
LV_DST_H += $(BUILD)/include/lv_conf.h $(BUILD)/include/v5lvgl.h

# Ensure headers are copied to build folder
$(BUILD)/include/%: %
	@echo "Copying $^ to $@"
ifeq ($(OS),Windows_NT)
	@if not exist "$(dir $@)" mkdir "$(dir $@)"
else
	@mkdir -p "$(dir $@)"
endif
	$(Q)$(call copyfile,$^,$@)

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
