#ifndef _LANGWELL_H_
#define _LANGWELL_H_

#include "drivers/gpio/gpio.h"



typedef struct lnw_gpio {
  struct gpio_chip chip;
  void *reg_base;
  void *reg_gplr;
  //spinlock_t lock;
  pci_device pdev;
  //struct irq_domain *domain;
  u32 (*get_flis_offset)(int gpio);
  u32 chip_irq_type;
  int type;
  //struct gpio_debug *debug;
} lnw_gpio_t;

extern lnw_gpio_t lnw_gpio;

void lnw_set_pininfo(struct lnw_gpio *lnw, unsigned gpio,
                     unsigned int type, const char *info);

#endif // _LANGWELL_H_
