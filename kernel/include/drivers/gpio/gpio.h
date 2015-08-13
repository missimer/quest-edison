#ifndef _GPIO_H_
#define _GPIO_H_

#include "util/list.h"
#include "drivers/pci/pci.h"
#include "util/printf.h"

struct gpio_chip {
  const char *label;
  struct device *dev;
  struct module *owner;
  struct list_head list;

  int (*request)(struct gpio_chip *chip,
                 unsigned offset);
  void (*free)(struct gpio_chip *chip,
               unsigned offset);
  int (*get_direction)(struct gpio_chip *chip,
                       unsigned offset);
  int (*direction_input)(struct gpio_chip *chip,
                         unsigned offset);
  int (*get)(struct gpio_chip *chip,
             unsigned offset);
  int (*direction_output)(struct gpio_chip *chip,
                          unsigned offset, int value);
  int (*set_debounce)(struct gpio_chip *chip,
                      unsigned offset, unsigned debounce);

  void (*set_pinmux)(int gpio, int alt);
  int (*get_pinmux)(int gpio);

  void (*set)(struct gpio_chip *chip,
              unsigned offset, int value);

  int (*to_irq)(struct gpio_chip *chip,
                unsigned offset);
#if 0
  void (*dbg_show)(struct seq_file *s,
                   struct gpio_chip *chip);
#endif
  int base;
  u16 ngpio;
  struct gpio_desc *desc;
  const char *const *names;
  unsigned can_sleep:1;
  unsigned exported:1;
};

struct gpio_control {
  unsigned type, num;
  char	 **pininfo;
  u32	reg, invert;
  u32 shift, rshift;
  u32	mask;
  int (*get)(struct gpio_control *control, void *private_data,
             unsigned gpio);
  int (*set)(struct gpio_control *control, void *private_data,
             unsigned gpio, unsigned int num);
};


#define TYPE_CONF_REG			0x00
#define TYPE_PIN_VALUE			0x01
#define TYPE_DIRECTION			0x02
#define TYPE_IRQ_TYPE			0x03
#define TYPE_PINMUX			0x04
#define TYPE_PULLMODE			0x05
#define TYPE_PULLSTRENGTH		0x06
#define TYPE_OPEN_DRAIN			0x07

#define TYPE_IRQ_COUNT			0x08
#define TYPE_WAKEUP			0x09
#define TYPE_WAKEUP_COUNT		0x0A
#define TYPE_OVERRIDE_OUTDIR		0x0B
#define TYPE_OVERRIDE_OUTVAL		0x0C
#define TYPE_OVERRIDE_INDIR		0x0D
#define TYPE_OVERRIDE_INVAL		0x0E
#define TYPE_DEBOUNCE			0x0F

#define TYPE_SBY_OVR_IO			0x10
#define TYPE_SBY_OVR_OUTVAL		0x11
#define TYPE_SBY_OVR_INVAL		0x12
#define TYPE_SBY_OVR_OUTDIR		0x13
#define TYPE_SBY_OVR_INDIR		0x14
#define TYPE_SBY_PUPD_STATE		0x15
#define TYPE_SBY_OD_DIS			0x16
#define TYPE_MAX			0x17



static inline struct gpio_control*
find_gpio_control(struct gpio_control *control, int num,
                  unsigned type)
{
  int i;

  for (i = 0; i < num; i++) {
    if ((control+i)->type == type)
      return control+i;
  }

  return NULL;
}

static inline int
find_pininfo_num(struct gpio_control *control, const char *info)
{
  int num = 0;

  while (num < control->num) {
    com1_printf("%d: %s\n", num, *(control->pininfo+num));
    if (!strcmp(*(control->pininfo+num), info))
      break;
    num++;
  }

  if (num < control->num)
    return num;

  com1_printf("%s failed\n", __FUNCTION__);
  return -1;
}


#endif // _GPIO_H_
