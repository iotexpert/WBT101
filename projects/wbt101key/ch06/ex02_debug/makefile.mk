APP_SRC +=  ex01_debug.c
C_FLAGS += -DSMUX_CHIP=$(CHIP)

ifdef DEBUG
C_FLAGS += -DDEBUG
C_FLAGS += -DSMUX_CHIP=$(CHIP)
APP_SRC += ex01_debug_pin_config.c
endif
