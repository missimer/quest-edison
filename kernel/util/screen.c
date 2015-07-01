/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
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

#include "smp/spinlock.h"
#include "util/screen.h"
#include "util/debug.h"

spinlock screen_lock = { 0 };

int
_putchar (int ch)
{
#ifdef INTEL_MID
  com1_printf("%c", (uint8) ch);
#else
  static int x, y;

  if (ch == '\n') {
    x = 0;
    y++;

    if (y > 24)
      y = 0;

    return (int) (uint8) ch;
  }

  pchVideo[y * 160 + x * 2] = ch;
  pchVideo[y * 160 + x * 2 + 1] = 7;
  x++;
#endif
  return (int) (uint8) ch;
}

int
putchar (int ch)
{
  int x;
  spinlock_lock (&screen_lock);
  x = _putchar (ch);
  spinlock_unlock (&screen_lock);
  return x;
}

int
print (char *pch)
{
  spinlock_lock (&screen_lock);
#ifdef INTEL_MID
  com1_printf("%s", pch);
#else
  while (*pch)
    _putchar (*pch++);
  spinlock_unlock (&screen_lock);
#endif
  return 0;
}


void
putx (uint32 l)
{

  int i, li;

  spinlock_lock (&screen_lock);
  for (i = 7; i >= 0; i--)
    if ((li = (l >> (i << 2)) & 0x0F) > 9)
      _putchar ('A' + li - 0x0A);
    else
      _putchar ('0' + li);
  spinlock_unlock (&screen_lock);
}

int
_print (char *pch)
{
  while (*pch)
    _putchar (*pch++);
  return 0;
}


void
_putx (uint32 l)
{

  int i, li;

  for (i = 7; i >= 0; i--)
    if ((li = (l >> (i << 2)) & 0x0F) > 9)
      _putchar ('A' + li - 0x0A);
    else
      _putchar ('0' + li);
}

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
