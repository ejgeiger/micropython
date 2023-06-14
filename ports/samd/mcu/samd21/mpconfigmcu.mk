CFLAGS_MCU += -mtune=cortex-m0plus -mcpu=cortex-m0plus -msoft-float

MPY_CROSS_MCU_ARCH = armv6m

MICROPY_HW_CODESIZE ?= 184K

ifeq ($(MICROPY_HW_CODESIZE), 248K)
FROZEN_MANIFEST ?= mcu/$(MCU_SERIES_LOWER)/manifest.py
endif

MICROPY_VFS_LFS1 ?= 1

SRC_S += shared/runtime/gchelper_thumb1.s
