# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0
#VERBOSE = 1

# include toolchain options
include vex/mkenv.mk

# location of the project source cpp and c files
SRC_C  = $(wildcard vpi/src/*.cpp) 
SRC_C += $(wildcard vpi/src/*.c)
SRC_C += $(wildcard vpi/src/*/*.cpp) 
SRC_C += $(wildcard vpi/src/*/*.c)
SRC_C += $(wildcard vpi/src/*/*/*.cpp) 
SRC_C += $(wildcard vpi/src/*/*/*.c)
SRC_C += $(wildcard vpi/src/*/*/*/*.cpp) 
SRC_C += $(wildcard vpi/src/*/*/*/*.c)

OBJA = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# location of include files that c and cpp files depend on
SRC_H  = $(wildcard vpi/include/*.h)
SRC_H += $(wildcard vpi/include/*/*.h)
SRC_H += $(wildcard vpi/include/*/*/*.h)
SRC_H += $(wildcard vpi/include/*/*/*/*.h)

# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include vpi/include lib/include

# headers needed to use library
VPI_SRC_H += $(wildcard $(SRC_H))

VPI_DST_H = $(addprefix $(BUILD)/, $(VPI_SRC_H:vpi/%=%))

#LVGL_LIB = lib/libv5lvgl.a

#LVGL_DST_LIB = $(addprefix $(BUILD)/, $(LVGL_LIB:lib/%=%))

#LVGL_H  = lib/include/v5lvgl.h lib/include/lv_conf.h
#LVGL_H += $(wildcard lib/include/lvgl/*.h)
#LVGL_H += $(wildcard lib/include/lvgl/src/*.h)
#LVGL_H += $(wildcard lib/include/lvgl/src/*/*.h)

#LVGL_DST_H = $(addprefix $(BUILD)/, $(LVGL_H:lib/%=%))

$(BUILD)/%: vpi/include/%
	$(Q)$(MKDIR)
	$(Q) $(call copyfile,$^, $@)

$(BUILD)/%: lib/%
	$(Q)$(MKDIR)
	$(Q) $(call copyfile,$^, $@)

$(BUILD)/%: %
	$(Q)$(MKDIR)
	$(Q) $(call copyfile,$^, $@)

$(BUILD)/lib/include/%: lib/include/%
	$(Q)$(MKDIR)
	$(Q) $(call copyfile,$^, $@)

#vpath %.h . vpi/ include/ vpi/include/ lib/ lib/include
vpath %.h . vpi/ include/ vpi/include/

# override default library name
PROJECTLIB = libvpi

# build targets
all: $(BUILD)/$(PROJECTLIB).a inc

# copy vpi header files
.PHONY: inc
inc: $(VPI_DST_H)
	$(ECHO) "Copy headers to build folder"

# include build rules
include vex/mkrules.mk
