// Deinitions common to all SAMD51 boards
#include "samd51.h"

#define MICROPY_CONFIG_ROM_LEVEL        (MICROPY_CONFIG_ROM_LEVEL_FULL_FEATURES)

// MicroPython emitters
#define MICROPY_EMIT_THUMB              (1)
#define MICROPY_EMIT_INLINE_THUMB       (1)

// Python internal features
#define MICROPY_ENABLE_EMERGENCY_EXCEPTION_BUF  (1)

#define MICROPY_FLOAT_IMPL              (MICROPY_FLOAT_IMPL_FLOAT)

#ifndef MICROPY_PY_BUILTINS_COMPLEX
#define MICROPY_PY_BUILTINS_COMPLEX     (0)
#endif

#ifndef MICROPY_PY_MATH
#define MICROPY_PY_MATH                 (1)
#define MP_NEED_LOG2                    (1)
#endif

#ifndef MICROPY_PY_CMATH
#define MICROPY_PY_CMATH                (0)
#endif
// Enable MD5 hash.
#define MICROPY_PY_UHASHLIB_MD5         (MICROPY_SSL_MBEDTLS)
#define MICROPY_TRACKED_ALLOC           (MICROPY_SSL_MBEDTLS)

#define MICROPY_PY_OS_SYNC              (1)
#define MICROPY_PY_OS_URANDOM           (1)
#define MICROPY_PY_RANDOM_SEED_INIT_FUNC (trng_random_u32())
unsigned long trng_random_u32(void);

#ifndef MICROPY_PY_MACHINE_PIN_BOARD_CPU
#define MICROPY_PY_MACHINE_PIN_BOARD_CPU (1)
#endif

#if MICROPY_PY_BLUETOOTH

#define MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE (1)
#define MICROPY_PY_BLUETOOTH_ENABLE_L2CAP_CHANNELS (1)
// Bluetooth code only runs in the scheduler, no locking/mutex required.
#define MICROPY_PY_BLUETOOTH_ENTER uint32_t atomic_state = 0;
#define MICROPY_PY_BLUETOOTH_EXIT (void)atomic_state;
#endif

// fatfs configuration used in ffconf.h
#define MICROPY_FATFS_ENABLE_LFN            (1)
#define MICROPY_FATFS_RPATH                 (2)
#define MICROPY_FATFS_MAX_SS                (4096)
#define MICROPY_FATFS_LFN_CODE_PAGE         437 /* 1=SFN/ANSI 437=LFN/U.S.(OEM) */

#define VFS_BLOCK_SIZE_BYTES            (1536) //

#ifndef MICROPY_HW_UART_TXBUF
#define MICROPY_HW_UART_TXBUF           (1)
#endif
#ifndef MICROPY_HW_UART_RTSCTS
#define MICROPY_HW_UART_RTSCTS          (1)
#endif

#define CPU_FREQ                        (120000000)
#define DFLL48M_FREQ                    (48000000)
#define MAX_CPU_FREQ                    (200000000)
#define DPLLx_REF_FREQ                  (32768)

#define NVIC_PRIORITYGROUP_4            ((uint32_t)0x00000003)
#define IRQ_PRI_PENDSV                  NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 7, 0)

static inline uint32_t raise_irq_pri(uint32_t pri) {
    uint32_t basepri = __get_BASEPRI();
    // If non-zero, the processor does not process any exception with a
    // priority value greater than or equal to BASEPRI.
    // When writing to BASEPRI_MAX the write goes to BASEPRI only if either:
    //   - Rn is non-zero and the current BASEPRI value is 0
    //   - Rn is non-zero and less than the current BASEPRI value
    pri <<= (8 - __NVIC_PRIO_BITS);
    __ASM volatile ("msr basepri_max, %0" : : "r" (pri) : "memory");
    return basepri;
}

// "basepri" should be the value returned from raise_irq_pri
static inline void restore_irq_pri(uint32_t basepri) {
    __set_BASEPRI(basepri);
}
