#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "drivers/serial/serial_reg.h"
#include "types.h"

#define BASE_BAUD         (1843200/16)
#define REAL_BAUD         (115200)
#define BAUD              (REAL_BAUD/24)
#define MMIO32_MEMBASE    (0xff010180)

#ifndef __ASSEMBLER__

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

#define DIV_ROUND_CLOSEST(x, divisor)(                  \
{                                                       \
        typeof(x) __x = x;                              \
        typeof(divisor) __d = divisor;                  \
        (((typeof(x))-1) > 0 ||                         \
         ((typeof(divisor))-1) > 0 || (__x) > 0) ?      \
                (((__x) + ((__d) / 2)) / (__d)) :       \
                (((__x) - ((__d) / 2)) / (__d));        \
}                                                       \
)

extern void initialize_serial_mmio32 (void);
extern void mmio32_putc (char);
extern int mmio32_getc (bool block);

#endif // __ASSEMBLER__

#endif // _SERIAL_H_
