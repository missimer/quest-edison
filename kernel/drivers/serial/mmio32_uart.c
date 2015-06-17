/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "drivers/serial/serial.h"
#include "types.h"
#include "mem/mem.h"
#include "kernel.h"

static uint32_t mmio_base = MMIO32_MEMBASE;

#define writel(val, addr) *((volatile uint32 *)addr) = val
#define writeb(val, addr) *((volatile uint8 *)addr) = val
#define readl(addr) (*((volatile uint32 *)addr))
#define readb(addr) (*((volatile uint8 *)addr))

static inline void
mmio32_out(uint32_t port, int offset, int value)
{
  if (offset > UART_MSR) {
    offset <<= 2;
    writel(value, port + offset);
  } else {
    unsigned char val = value & 0xff;
    writeb(val, port + offset);
  }
}

static inline unsigned int
mmio32_in(uint32_t port, int offset)
{
  unsigned int val;

  if (offset > UART_MSR) {
    offset <<= 2;
    val = readl(port + offset);
  }
  else {
    val = (unsigned int)readb(port + offset);
  }

  return val;
}

static void
wait_for_xmitr (uint32_t port)
{
  unsigned int status;

  for (;;) {
    status = mmio32_in (port, UART_LSR);
    if ((status & BOTH_EMPTY) == BOTH_EMPTY) {
      return;
    }
    /* asm volatile ("rep;nop": : :"memory"); */
    asm volatile("pause");
  }
}

static void
wait_for_rcvrr (uint32_t port)
{
  unsigned int status;

  for (;;) {
    status = mmio32_in (port, UART_LSR);
    if ((status & UART_LSR_DR) == UART_LSR_DR) {
      return;
    }
    asm volatile("pause");
    /* asm volatile ("rep;nop": : :"memory"); */
  }
}

static void
serial_putc (uint32_t port, int c)
{
  wait_for_xmitr (port);
  mmio32_out (port, UART_TX, c);
}

void
mmio32_putc (char c)
{
  if (c == '\n') serial_putc (mmio_base, '\r');
  serial_putc (mmio_base, c);
}

static int
serial_getc (uint32_t port)
{
  wait_for_rcvrr (port);
  return mmio32_in (port, UART_RX);
}

int
mmio32_getc ()
{
  return serial_getc(mmio_base);
}

unsigned int
probe_baud(uint32_t port)
{
  unsigned char lcr, dll, dlm;
  unsigned int quot;

  lcr = mmio32_in (port, UART_LCR);
  mmio32_out (port, UART_LCR, lcr | UART_LCR_DLAB);
  dll = mmio32_in (port, UART_DLL);
  dlm = mmio32_in (port, UART_DLM);
  mmio32_out (port, UART_LCR, lcr);

  quot = (dlm << 8) | dll;
  return (BASE_BAUD) / quot;
}

void
initialize_serial_mmio32 (void)
{
  uint32_t port;
  unsigned int divisor;
  unsigned char c;

  BITMAP_SET(mm_table, MMIO32_MEMBASE >> 12);
  mmio_base = (uint32)map_virtual_page((MMIO32_MEMBASE & 0xFFFFF000) | 3);

  if(mmio_base == 0) {
    panic("Failed to setup mmio_base");
  }
  mmio_base |= (0xFFF & MMIO32_MEMBASE);

  port = mmio_base;

  mmio32_out (port, UART_LCR, 0x3);      /* 8n1 */
  mmio32_out (port, UART_IER, 0);        /* no interrupt */
  /* XXX: enable FIFO? */
  mmio32_out (port, UART_FCR, 0);        /* no fifo */
  mmio32_out (port, UART_MCR, 0x3);      /* DTR + RTS */

  divisor = DIV_ROUND_CLOSEST (BASE_BAUD * 16, 16 * probe_baud (mmio_base));
  /* divisor = DIV_ROUND_CLOSEST (BASE_BAUD * 16, 16 * BAUD); */
  c = mmio32_in (port, UART_LCR);
  mmio32_out (port, UART_LCR, c | UART_LCR_DLAB);
  mmio32_out (port, UART_DLL, divisor & 0xff);
  mmio32_out (port, UART_DLM, (divisor >> 8) & 0xff);
  mmio32_out (port, UART_LCR, c & ~UART_LCR_DLAB);
}


/* vi: set et sw=2 sts=2: */
