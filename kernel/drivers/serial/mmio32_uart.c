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
#include "drivers/pci/pci.h"
#include "util/debug.h"

#define DEBUG_MMIO32_UART

#ifdef DEBUG_MMIO32_UART
#define DLOG(fmt,...) DLOG_PREFIX("mmio32-uart",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static uint32_t mmio_base = 0;

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
    asm volatile("pause");
  }
}

static bool
wait_for_rcvrr (uint32_t port, bool block)
{
  unsigned int status;

  for (;;) {
    status = mmio32_in (port, UART_LSR);
    if ((status & UART_LSR_DR) == UART_LSR_DR) {
      return TRUE;
    }
    if(block) {
      asm volatile("pause");
    }
    else {
      return FALSE;
    }
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
  if(mmio_base) {
    if (c == '\n') serial_putc (mmio_base, '\r');
    serial_putc (mmio_base, c);
  }
}

static int
serial_getc(uint32_t port, bool block)
{
  if(wait_for_rcvrr (port, block)) {
    return mmio32_in (port, UART_RX);
  }
  else {
    return -1;
  }
}

int
mmio32_getc(bool block)
{
  if(mmio_base) {
    return serial_getc(mmio_base, block);
  }
  else {
    return 0;
  }
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
serial_mmio32_manual_init(uint32 phys_addr)
{
  uint32_t port;
  unsigned int divisor;
  unsigned char c;

  BITMAP_SET(mm_table, phys_addr >> 12);
  if(mmio_base) {
    unmap_virtual_page((void*)mmio_base);
  }
  mmio_base = (uint32)map_virtual_page((phys_addr & 0xFFFFF000) | 0x13);

  if(mmio_base == 0) {
    panic("Failed to setup mmio_base");
  }
  mmio_base |= (0xFFF & phys_addr);

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

static uint32
mmio32_irq_handler(uint8 vec)
{
  panic("mmio32_irq_handler not implemented");
}

static bool
serial_mmio32_init(void)
{
  uint i, device_index, irq_pin;
  pci_device serial_device;
  pci_irq_t irq;
  int bus, dev, func;
  uint irq_line;
  uint32 bar0;

  if (mp_ISA_PC) {
    DLOG ("Cannot operate without PCI");
    return FALSE;
  }

  /* Find the serial device on the PCI bus */
  device_index = ~0;
  i=0;
  while (pci_find_device (0xFFFF, 0xFFFF, 0x07, 0x00, i, &i)) {
    if (pci_get_device (i, &serial_device)) {
      if (serial_device.progIF == 0x02) {
        device_index = i;
        break;
      }
      i++;
    } else break;
  }

  if (device_index == ~0) {
    DLOG ("Unable to find compatible device on PCI bus");
    return FALSE;
  }

  device_index += 3;

  if (!pci_get_device (device_index, &serial_device)) {
    DLOG ("Unable to get PCI device from PCI subsystem");
    return FALSE;
  }

  if(serial_device.classcode != 0x07 ||
     serial_device.subclass  != 0x00 ||
     serial_device.progIF    != 0x02 ||
     serial_device.func      != 0x03) {
    return FALSE;
  }

  bus = serial_device.bus;
  dev = serial_device.slot;
  func = serial_device.func;

  /* DLOG ("Using PCI bus=%x dev=%x func=%x", bus, dev, func); */

  /* if (!pci_get_interrupt (device_index, &irq_line, &irq_pin)) { */
  /*   DLOG ("Unable to get IRQ"); */
  /*   return FALSE; */
  /* } */

  /* DLOG ("Using IRQ pin=%X", irq_pin); */

  /* if (pci_irq_find (bus, dev, irq_pin, &irq)) { */
  /*   /\* use PCI routing table *\/ */
  /*   DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi); */
  /*   if (!pci_irq_map_handler (&irq, mmio32_irq_handler, 0x01, */
  /*                             IOAPIC_DESTINATION_LOGICAL, */
  /*                             IOAPIC_DELIVERY_FIXED)) { */
  /*     DLOG ("Unable to map IRQ handler"); */
  /*     return FALSE; */
  /*   } */
  /*   irq_line = irq.gsi; */
  /* } */

  if (!pci_decode_bar (device_index, 0, &bar0, NULL, NULL)) {
    DLOG ("unable to decode BAR0");
    return FALSE;
  }

  serial_mmio32_manual_init(bar0);

  DLOG("Done with serial init");

  return TRUE;

}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = serial_mmio32_init
};

DEF_MODULE (mm_serial, "Memory Mapped UART driver", &mod_ops, {"pci"});


/* vi: set et sw=2 sts=2: */
