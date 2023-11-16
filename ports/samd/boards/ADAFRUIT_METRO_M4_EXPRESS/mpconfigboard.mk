MCU_SERIES = SAMD51
CMSIS_MCU = SAMD51J19A
LD_FILES = boards/samd51x19a.ld sections.ld
TEXT0 = 0x4000

# MicroPython settings
MICROPY_HW_CODESIZE ?= 496K

BOARD_VARIANT ?= WLAN
