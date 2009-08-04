#include "drivers/input/keyboard.h"
#include "arch/i386.h"
#include "sched/sched.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "util/circular.h"
#include "util/printf.h"

static char lcase_scancode[128] =
    "\0\e1234567890-=\177\tqwertyuiop[]\n\0asdfghjkl;'`\0\\zxcvbnm,./\0*\0 \0\0\0\0\0\0\0\0\0\0\0\0\000789-456+1230.\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
static char ucase_scancode[128] =
    "\0\e1234567890-=\177\tQWERTYUIOP[]\n\0ASDFGHJKL;'`\0\\ZXCVBNM,./\0*\0 \0\0\0\0\0\0\0\0\0\0\0\0\000789-456+1230.\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

static circular keyb_buffer;
static uint8 buffer_space[KEYBOARD_BUFFER_SIZE];

static bool escaped, shifted;

static uint32
kbd_irq_handler (uint8 vec)
{
  uint8 ch;
  uint8 code;

  code = inb (KEYBOARD_DATA_PORT);

  if (escaped) {
    code += 0x100;
    escaped = FALSE;
  }

  switch (code) {
  case 0x2A:
  case 0x36:
    shifted = TRUE;
    break;
  case 0xAA:
  case 0xB6:
    shifted = FALSE;
    break;
  case 0xE0:
    escaped = TRUE;
    break;
  default:

    if (!(code & 0x80)) {
      /* not an oob or released keystroke */
      if (shifted)
        ch = ucase_scancode[(int) code];
      else
        ch = lcase_scancode[(int) code];
      if (circular_insert_nowait (&keyb_buffer, &ch) < 0) {
        lock_kernel();
        com1_printf ("keyboard_8042: dropped keystroke: %X (%c)\n", code, ch);
        unlock_kernel();
      }
    }

    break;
  }

  return 0;
}

void
init_keyboard_8042 (void)
{
  escaped = shifted = FALSE;

  circular_init (&keyb_buffer, 
                 (void *)buffer_space, 
                 KEYBOARD_BUFFER_SIZE, 
                 sizeof (uint8));

  if (mp_ISA_PC) {
    set_vector_handler (KEYBOARD_IRQ, kbd_irq_handler);
  } else {
    IOAPIC_map_GSI (IRQ_to_GSI (mp_ISA_bus_id, KEYBOARD_IRQ),
                    KEYBOARD_VECTOR, 0xFF00000000000800LL);
    set_vector_handler (KEYBOARD_VECTOR, kbd_irq_handler);
  }
}

uint8
keyboard_8042_next (void)
{
  uint8 ch;
  circular_remove (&keyb_buffer, &ch);
  return ch;
}
