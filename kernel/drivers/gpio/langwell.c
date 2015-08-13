/* langwell.c Moorestown platform Langwell chip GPIO driver
 * Copyright (c) 2015,  Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* This file is based off of the Linux Langwell chip GPIO driver.  The original
 *  Linux license is below:
 */
/* gpio-langwell.c Moorestown platform Langwell chip GPIO driver
 * Copyright (c) 2008 - 2013,  Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "types.h"
#include "mem/mem.h"
#include "kernel.h"
#include "drivers/pci/pci.h"
#include "util/debug.h"
#include "drivers/gpio/gpio.h"
#include "drivers/gpio/langwell.h"
#include "drivers/gpio/intel_scu_flis.h"

#define DEBUG_LANGWELL_GPIO

#ifdef DEBUG_LANGWELL_GPIO
#define DLOG(fmt,...) DLOG_PREFIX("langwell-gpio",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define TANGIER_GPLR_OFFSET 4

#define to_lnw_priv(chip)	container_of(chip, struct lnw_gpio, chip)
#define __iomem
#define WARN(x, y, z) x
#define spin_lock_irqsave(x, y)
#define spin_unlock_irqrestore(x, y)
#define dev_err(x, y, ...) DLOG(y, ##__VA_ARGS__)

#define EINVAL 1

#define UNUSED __attribute__((unused))

static char *pinvalue[] = {"low", "high"};
static char *pindirection[] = {"in", "out"};
static char *irqtype[] = {"irq_none", "edge_rising", "edge_falling",
			"edge_both"};
static char *pinmux[] = {"mode0", "mode1", "mode2", "mode3", "mode4", "mode5",
			"mode6", "mode7"};
static char *pullmode[] = {"nopull", "pullup", "pulldown"};
static char *pullstrength[] = {"2k", "20k", "50k", "910ohms"};
static char *enable[] = {"disable", "enable"};
static char *override_direction[] = {"no-override", "override-enable",
			"override-disable"};
static char *override_value[] = {"no-override", "override-high",
			"override-low"};
static char *standby_trigger[] = {"no-override", "override-trigger",
			"override-notrigger"};
static char *standby_pupd_state[] = {"keep", "pulldown", "pullup", "nopull"};

static void lnw_gpio_set(struct gpio_chip *chip, unsigned offset, int value);
static inline bool is_merr_i2c_flis(u32 offset);


enum GPIO_CONTROLLERS {
  LINCROFT_GPIO,
  PENWELL_GPIO_AON,
  PENWELL_GPIO_CORE,
  CLOVERVIEW_GPIO_AON,
  CLOVERVIEW_GPIO_CORE,
  TANGIER_GPIO,
};

enum GPIO_REG {
  GPLR = 0, /* pin level read-only */
  GPDR, /* pin direction */
  GPSR, /* pin set */
  GPCR, /* pin clear */
  GRER, /* rising edge detect */
  GFER, /* falling edge detect */
  GEDR, /* edge detect result */
  GAFR, /* alt function */
  GFBR = 9, /* glitch filter bypas */
  GPIT, /* interrupt type */
  GPIP = GFER, /* level interrupt polarity */
  GPIM = GRER, /* level interrupt mask */

  /* the following registers only exist on MRFLD */
  GFBR_TNG = 6,
  GIMR, /* interrupt mask */
  GISR, /* interrupt source */
  GITR = 32, /* interrupt type */
  GLPR = 33, /* level-input polarity */
};


static void __iomem *gpio_reg(struct gpio_chip *chip, unsigned offset,
                              enum GPIO_REG reg_type)
{
  struct lnw_gpio *lnw = to_lnw_priv(chip);
  unsigned nreg = chip->ngpio / 32;
  u8 reg = offset / 32;
  void __iomem *ptr;
  void *base;

  /**
   * On TNG B0, GITR[0]'s address is 0xFF008300, while GPLR[0]'s address
   * is 0xFF008004. To count GITR[0]'s address, it's easier to count
   * from 0xFF008000. So for GITR,GLPR... we switch the base to reg_base.
   * This does not affect PNW/CLV, since the reg_gplr is the reg_base,
   * while on TNG, the reg_gplr has an offset of 0x4.
   */
  base = reg_type < GITR ? lnw->reg_gplr : lnw->reg_base;
  ptr = (void __iomem *)(base + reg_type * nreg * 4 + reg * 4);
  return ptr;
}


static int lnw_gpio_get(struct gpio_chip *chip, unsigned offset)
{
  void __iomem *gplr = gpio_reg(chip, offset, GPLR);

  return readl(gplr) & BIT(offset % 32);
}

static int gpio_get_pinvalue(struct gpio_control *control, void *private_data,
		unsigned gpio)
{
	struct lnw_gpio *lnw = private_data;
	u32 value;


	value = lnw_gpio_get(&lnw->chip, gpio);

	return value ? 1 : 0;
}

static int gpio_set_pinvalue(struct gpio_control *control, void *private_data,
		unsigned gpio, unsigned int num)
{
	struct lnw_gpio *lnw = private_data;


	lnw_gpio_set(&lnw->chip, gpio, num);
	return 0;
}

static int gpio_get_normal(struct gpio_control *control, void *private_data,
		unsigned gpio)
{
	struct lnw_gpio *lnw = private_data;
	u32 __iomem *mem;
	u32 value;


	mem = gpio_reg(&lnw->chip, gpio, control->reg);

	value = readl(mem);
	value &= BIT(gpio % 32);

	if (control->invert)
		return value ? 0 : 1;
	else
		return value ? 1 : 0;
}

static int gpio_set_normal(struct gpio_control *control, void *private_data,
                           unsigned gpio, unsigned int num)
{
	struct lnw_gpio *lnw = private_data;
	u32 __iomem *mem;
	u32 value;
	__attribute__((unused)) unsigned long flags;


	mem = gpio_reg(&lnw->chip, gpio, control->reg);

	spin_lock_irqsave(&lnw->lock, flags);
	value = readl(mem);
	value &= ~BIT(gpio % 32);
	if (control->invert) {
		if (num)
			value &= ~BIT(gpio % 32);
		else
			value |= BIT(gpio % 32);
	} else {
		if (num)
			value |= BIT(gpio % 32);
		else
			value &= ~BIT(gpio % 32);
	}
	writel(value, mem);
	spin_unlock_irqrestore(&lnw->lock, flags);

	return 0;
}

static int gpio_get_irqtype(struct gpio_control *control, void *private_data,
		unsigned gpio)
{
	struct lnw_gpio *lnw = private_data;
	void __iomem *grer = gpio_reg(&lnw->chip, gpio, GRER);
	void __iomem *gfer = gpio_reg(&lnw->chip, gpio, GFER);
	u32 value;
	int num;


	value = readl(grer) & BIT(gpio % 32);
	num = value ? 1 : 0;
	value = readl(gfer) & BIT(gpio % 32);
	if (num)
		num = value ? 3 : 1;
	else
		num = value ? 2 : 0;

	return num;
}

static int flis_get_normal(struct gpio_control *control, void *private_data,
		unsigned gpio)
{
	struct lnw_gpio *lnw = private_data;
	u32 offset, value;
	int num;


	if (lnw->type == TANGIER_GPIO) {
		offset = lnw->get_flis_offset(gpio);
		if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
			return -1;

		value = get_flis_value(offset);
		num = (value >> control->shift) & control->mask;
		if (num < control->num)
			return num;
	}

	return -1;
}

static int flis_set_normal(struct gpio_control *control, void *private_data,
		unsigned gpio, unsigned int num)
{
	struct lnw_gpio *lnw = private_data;
	u32 shift = control->shift;
	u32 mask = control->mask;
	u32 offset, value;
	__attribute__((unused)) unsigned long flags;

	if (lnw->type == TANGIER_GPIO) {
		offset = lnw->get_flis_offset(gpio);
		if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
			return -1;

		if (!is_merr_i2c_flis(offset))
			spin_lock_irqsave(&lnw->lock, flags);
		value = get_flis_value(offset);
		value &= ~(mask << shift);
		value |= ((num & mask) << shift);
		set_flis_value(value, offset);
		if (!is_merr_i2c_flis(offset))
			spin_unlock_irqrestore(&lnw->lock, flags);
		return 0;
	}
        panic("flis_set_normal failed\n");
	return -1;
}

static int flis_get_override(struct gpio_control *control, void *private_data,
		unsigned gpio)
{
	struct lnw_gpio *lnw = private_data;
	u32 offset, value;
	u32 val_bit, en_bit;
	int num;


	if (lnw->type == TANGIER_GPIO) {
		offset = lnw->get_flis_offset(gpio);
		if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
			return -1;

		val_bit = 1 << control->shift;
		en_bit = 1 << control->rshift;

		value = get_flis_value(offset);

		if (value & en_bit)
			if (value & val_bit)
				num = 1;
			else
				num = 2;
		else
			num = 0;

		return num;
	}

	return -1;
}

static int flis_set_override(struct gpio_control *control, void *private_data,
		unsigned gpio, unsigned int num)
{
	struct lnw_gpio *lnw = private_data;
	u32 offset, value;
	u32 val_bit, en_bit;
	__attribute__((unused)) unsigned long flags;


	if (lnw->type == TANGIER_GPIO) {
		offset = lnw->get_flis_offset(gpio);
		if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
			return -1;

		val_bit = 1 << control->shift;
		en_bit = 1 << control->rshift;

		if (!is_merr_i2c_flis(offset))
			spin_lock_irqsave(&lnw->lock, flags);
		value = get_flis_value(offset);
		switch (num) {
		case 0:
			value &= ~(en_bit | val_bit);
			break;
		case 1:
			value |= (en_bit | val_bit);
			break;
		case 2:
			value |= en_bit;
			value &= ~val_bit;
			break;
		default:
			break;
		}
		set_flis_value(value, offset);
		if (!is_merr_i2c_flis(offset))
			spin_unlock_irqrestore(&lnw->lock, flags);

		return 0;
	}

	return -1;
}

#define GPIO_VALUE_CONTROL(xtype, xinfo, xnum) \
{	.type = xtype, .pininfo = xinfo, .num = xnum, \
	.get = gpio_get_pinvalue, .set = gpio_set_pinvalue}
#define GPIO_NORMAL_CONTROL(xtype, xinfo, xnum, xreg, xinvert) \
{	.type = xtype, .pininfo = xinfo, .num = xnum, .reg = xreg, \
	.invert = xinvert, .get = gpio_get_normal, .set = gpio_set_normal}
#define GPIO_IRQTYPE_CONTROL(xtype, xinfo, xnum) \
{	.type = xtype, .pininfo = xinfo, .num = xnum, \
	.get = gpio_get_irqtype, .set = NULL}
#define FLIS_NORMAL_CONTROL(xtype, xinfo, xnum, xshift, xmask) \
{	.type = xtype, .pininfo = xinfo, .num = xnum, .shift = xshift, \
	.mask = xmask, .get = flis_get_normal, .set = flis_set_normal}
#define FLIS_OVERRIDE_CONTROL(xtype, xinfo, xnum, xshift, xrshift) \
{	.type = xtype, .pininfo = xinfo, .num = xnum, .shift = xshift, \
	.rshift = xrshift, .get = flis_get_override, .set = flis_set_override}

static struct gpio_control lnw_gpio_controls[] = {
  GPIO_VALUE_CONTROL(TYPE_PIN_VALUE, pinvalue, 2),
  GPIO_NORMAL_CONTROL(TYPE_DIRECTION, pindirection, 2, GPDR, 0),
  GPIO_IRQTYPE_CONTROL(TYPE_IRQ_TYPE, irqtype, 4),
  GPIO_NORMAL_CONTROL(TYPE_DEBOUNCE, enable, 2, GFBR_TNG, 1),
  FLIS_NORMAL_CONTROL(TYPE_PINMUX, pinmux, 8, 0, 0x7),
  FLIS_NORMAL_CONTROL(TYPE_PULLSTRENGTH, pullstrength, 4, 4, 0x7),
  FLIS_NORMAL_CONTROL(TYPE_PULLMODE, pullmode, 3, 8, 0x3),
  FLIS_NORMAL_CONTROL(TYPE_OPEN_DRAIN, enable, 2, 21, 0x1),
  FLIS_OVERRIDE_CONTROL(TYPE_OVERRIDE_INDIR, override_direction, 3, 12, 13),
  FLIS_OVERRIDE_CONTROL(TYPE_OVERRIDE_OUTDIR, override_direction, 3, 14, 15),
  FLIS_OVERRIDE_CONTROL(TYPE_OVERRIDE_INVAL, override_value, 3, 16, 17),
  FLIS_OVERRIDE_CONTROL(TYPE_OVERRIDE_OUTVAL, override_value, 3, 18, 19),
  FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_IO, standby_trigger, 3, 23, 22),
  FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_OUTVAL, override_value, 3, 18, 24),
  FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_INVAL, override_value, 3, 16, 25),
  FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_OUTDIR, override_direction, 3, 14, 26),
  FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_INDIR, override_direction, 3, 12, 27),
  FLIS_NORMAL_CONTROL(TYPE_SBY_PUPD_STATE, standby_pupd_state, 4, 28, 0x3),
  FLIS_NORMAL_CONTROL(TYPE_SBY_OD_DIS, enable, 2, 30, 0x1),
};


struct lnw_gpio_ddata_t {
  u16 ngpio;         /* number of gpio pins */
  u32 gplr_offset;   /* offset of first GPLR register from base */
  u32 (*get_flis_offset)(int gpio);
  u32 chip_irq_type; /* chip interrupt type */
};

#define IRQ_TYPE_EDGE	(1 << 0)
#define IRQ_TYPE_LEVEL	(1 << 1)

#define PULLUP_ENABLE	(1 << 8)
#define PULLDOWN_ENABLE	(1 << 9)
#define PUPD_VAL_2K	(0 << 4)
#define PUPD_VAL_20K	(1 << 4)
#define PUPD_VAL_50K	(2 << 4)
#define PUPD_VAL_910	(3 << 4)


struct gpio_flis_pair {
  int gpio; /* gpio number */
  int offset; /* register offset from FLIS base */
};

/*
 * The following mapping table lists the pin and flis offset pair
 * of some key gpio pins, the offset of other gpios can be calculated
 * from the table.
 */
static struct gpio_flis_pair gpio_flis_mapping_table[] = {
  { 0, 0x2900 },
  { 12, 0x2544 },
  { 14, 0x0958 },
  { 16, 0x2D18 },
  { 17, 0x1D10 },
  { 19, 0x1D00 },
  { 23, 0x1D18 },
  { 31, -EINVAL }, /* No GPIO 31 in pin list */
  { 32, 0x1508 },
  { 44, 0x3500 },
  { 64, 0x2534 },
  { 68, 0x2D1C },
  { 70, 0x1500 },
  { 72, 0x3D00 },
  { 77, 0x0D00 },
  { 97, 0x0954 },
  { 98, -EINVAL }, /* No GPIO 98-101 in pin list */
  { 102, 0x1910 },
  { 120, 0x1900 },
  { 124, 0x2100 },
  { 136, -EINVAL }, /* No GPIO 136 in pin list */
  { 137, 0x2D00 },
  { 143, -EINVAL }, /* No GPIO 143-153 in pin list */
  { 154, 0x092C },
  { 164, 0x3900 },
  { 177, 0x2500 },
  { 190, 0x2D50 },
};

static void __iomem *gpio_reg_2bit(struct gpio_chip *chip, unsigned offset,
                                   enum GPIO_REG reg_type)
{
  struct lnw_gpio *lnw = to_lnw_priv(chip);
  unsigned nreg = chip->ngpio / 32;
  u8 reg = offset / 16;
  void __iomem *ptr;

  ptr = (void __iomem *)(lnw->reg_base + reg_type * nreg * 4 + reg * 4);
  return ptr;
}

static int lnw_gpio_request(struct gpio_chip *chip, unsigned offset)
{
  struct lnw_gpio *lnw = to_lnw_priv(chip);
  u32 value;
  void __iomem *gafr;
  int shift, af;

  if (lnw->type > CLOVERVIEW_GPIO_CORE)
    return 0;

  gafr = gpio_reg_2bit(chip, offset, GAFR);
  value = readl(gafr);
  shift = (offset % 16) << 1;
  af = (value >> shift) & 3;

  if (af) {
    value &= ~(3 << shift);
    writel(value, gafr);
  }
  return 0;
}

static u32 get_flis_offset_by_gpio(int gpio)
{
  int i;
  int start;
  u32 offset = -1;

  for (i = 0; i < ARRAY_SIZE(gpio_flis_mapping_table) - 1; i++) {
    if (gpio >= gpio_flis_mapping_table[i].gpio
        && gpio < gpio_flis_mapping_table[i + 1].gpio)
      break;
  }

  start = gpio_flis_mapping_table[i].gpio;

  if (gpio_flis_mapping_table[i].offset != -EINVAL) {
    offset = gpio_flis_mapping_table[i].offset
      + (gpio - start) * 4;
  }

  return offset;
}

static struct lnw_gpio_ddata_t lnw_gpio_ddata[] = {
  [LINCROFT_GPIO] = {
    .ngpio = 64,
  },
  [PENWELL_GPIO_AON] = {
    .ngpio = 96,
    .chip_irq_type = IRQ_TYPE_EDGE,
  },
  [PENWELL_GPIO_CORE] = {
    .ngpio = 96,
    .chip_irq_type = IRQ_TYPE_EDGE,
  },
  [CLOVERVIEW_GPIO_AON] = {
    .ngpio = 96,
    .chip_irq_type = IRQ_TYPE_EDGE | IRQ_TYPE_LEVEL,
  },
  [CLOVERVIEW_GPIO_CORE] = {
    .ngpio = 96,
    .chip_irq_type = IRQ_TYPE_EDGE,
  },
  [TANGIER_GPIO] = {
    .ngpio = 192,
    .gplr_offset = 4,
    .get_flis_offset = get_flis_offset_by_gpio,
    .chip_irq_type = IRQ_TYPE_EDGE | IRQ_TYPE_LEVEL,
  },
};

lnw_gpio_t lnw_gpio;

#define TANGIER_I2C_FLIS_START 0x1D00
#define TANGIER_I2C_FLIS_END   0x1D34

static inline bool is_merr_i2c_flis(u32 offset)
{
  return ((offset >= TANGIER_I2C_FLIS_START)
          && (offset <= TANGIER_I2C_FLIS_END));
}

static int lnw_gpio_set_pull(struct gpio_chip *chip, unsigned gpio, int value)
{
  u32 flis_offset, flis_value;
  struct lnw_gpio *lnw = to_lnw_priv(chip);
  //unsigned long flags;

  if (lnw->type != TANGIER_GPIO)
    return 0;

  flis_offset = lnw->get_flis_offset(gpio);
  if (flis_offset == -EINVAL)
    return -EINVAL;
  if (is_merr_i2c_flis(flis_offset))
    return 0;

  //spin_lock_irqsave(&lnw->lock, flags);
  flis_value = get_flis_value(flis_offset);
  if (value) {
    flis_value |= PULLUP_ENABLE;
    flis_value &= ~PULLDOWN_ENABLE;
  } else {
    flis_value |= PULLDOWN_ENABLE;
    flis_value &= ~PULLUP_ENABLE;
  }
  flis_value |= PUPD_VAL_50K;
  set_flis_value(flis_value, flis_offset);
  //spin_unlock_irqrestore(&lnw->lock, flags);

  return 0;
}

static void lnw_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
  void __iomem *gpsr, *gpcr;

  lnw_gpio_set_pull(chip, offset, value);

  if (value) {
    gpsr = gpio_reg(chip, offset, GPSR);
    writel(BIT(offset % 32), gpsr);
  } else {
    gpcr = gpio_reg(chip, offset, GPCR);
    writel(BIT(offset % 32), gpcr);
  }
}

static void lnw_irq_init_hw(struct lnw_gpio *lnw)
{
  void *reg;
  unsigned base;

  for (base = 0; base < lnw->chip.ngpio; base += 32) {
    /* Clear the rising-edge detect register */
    reg = gpio_reg(&lnw->chip, base, GRER);
    writel(0, reg);
    /* Clear the falling-edge detect register */
    reg = gpio_reg(&lnw->chip, base, GFER);
    writel(0, reg);
    /* Clear the edge detect status register */
    reg = gpio_reg(&lnw->chip, base, GEDR);
    writel(~0, reg);
  }
}

int gpio_get_alt(int gpio)
{
  struct lnw_gpio *lnw;
  u32 __iomem *mem;
  int reg;
  int bit;
  u32 value;
  u32 offset;

  /* use this trick to get memio */
  lnw = &lnw_gpio;
  if (gpio < lnw->chip.base || gpio >= lnw->chip.base + lnw->chip.ngpio) {
    dev_err(lnw->chip.dev,
            "langwell_gpio: wrong pin %d to config alt\n", gpio);
    return -1;
  }
#if 0
  if (lnw->irq_base + gpio - lnw->chip.base != gpio_to_irq(gpio)) {
    dev_err(lnw->chip.dev,
            "langwell_gpio: wrong chip data for pin %d\n", gpio);
    return -1;
  }
#endif
  gpio -= lnw->chip.base;

  if (lnw->type != TANGIER_GPIO) {
    reg = gpio / 16;
    bit = gpio % 16;

    mem = gpio_reg(&lnw->chip, 0, GAFR);
    value = readl(mem + reg);
    value &= (3 << (bit * 2));
    value >>= (bit * 2);
  } else {
    offset = lnw->get_flis_offset(gpio);
    if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
      return -EINVAL;

    value = get_flis_value(offset) & 7;
  }

  return value;
}

static int lnw_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
  panic("lnw_gpio_to_irq not implemented");
}


void lnw_gpio_set_alt(int gpio, int alt)
{
  struct lnw_gpio *lnw;
  u32 __iomem *mem;
  int reg;
  int bit;
  u32 offset;
  u32 value;
  //unsigned long flags;

  /* use this trick to get memio */
  lnw = &lnw_gpio;

  if (gpio < lnw->chip.base || gpio >= lnw->chip.base + lnw->chip.ngpio) {
    DLOG("langwell_gpio: wrong pin %d to config alt\n", gpio);
    return;
  }
#if 0
  if (lnw->irq_base + gpio - lnw->chip.base != gpio_to_irq(gpio)) {
    dev_err(lnw->chip.dev, "langwell_gpio: wrong chip data for pin %d\n", gpio);
    return;
  }
#endif
  gpio -= lnw->chip.base;

  if (lnw->type != TANGIER_GPIO) {
    reg = gpio / 16;
    bit = gpio % 16;

    mem = gpio_reg(&lnw->chip, 0, GAFR);
    //spin_lock_irqsave(&lnw->lock, flags);
    value = readl(mem + reg);
    value &= ~(3 << (bit * 2));
    value |= (alt & 3) << (bit * 2);
    writel(value, mem + reg);
    //spin_unlock_irqrestore(&lnw->lock, flags);
    DLOG("ALT: writing 0x%x to %p\n",
         value, mem + reg);
  } else {
    offset = lnw->get_flis_offset(gpio);
    if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
      return;

    if (!is_merr_i2c_flis(offset))
      spin_lock_irqsave(&lnw->lock, flags);

    value = get_flis_value(offset);
    value &= ~7;
    value |= (alt & 7);
    set_flis_value(value, offset);

    if (!is_merr_i2c_flis(offset))
      spin_unlock_irqrestore(&lnw->lock, flags);
  }
}

static int lnw_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
                                 unsigned debounce)
{
  struct lnw_gpio *lnw = to_lnw_priv(chip);
  void __iomem *gfbr;
  //unsigned long flags;
  u32 value;
  enum GPIO_REG reg_type;

  reg_type = (lnw->type == TANGIER_GPIO) ? GFBR_TNG : GFBR;
  gfbr = gpio_reg(chip, offset, reg_type);

  /* if (lnw->pdev) */
  /*   pm_runtime_get(&lnw->pdev->dev); */

  spin_lock_irqsave(&lnw->lock, flags);
  value = readl(gfbr);
  if (debounce) {
    /* debounce bypass disable */
    value &= ~BIT(offset % 32);
  } else {
    /* debounce bypass enable */
    value |= BIT(offset % 32);
  }
  writel(value, gfbr);
  spin_unlock_irqrestore(&lnw->lock, flags);

  /* if (lnw->pdev) */
  /*   pm_runtime_put(&lnw->pdev->dev); */

  return 0;
}


static int lnw_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
  //struct lnw_gpio *lnw = to_lnw_priv(chip);
  void __iomem *gpdr = gpio_reg(chip, offset, GPDR);
  u32 value;
  //unsigned long flags;

  //if (lnw->pdev)
  //  pm_runtime_get(&lnw->pdev->dev);

  //spin_lock_irqsave(&lnw->lock, flags);
  value = readl(gpdr);
  value &= ~BIT(offset % 32);
  writel(value, gpdr);
  //spin_unlock_irqrestore(&lnw->lock, flags);

  //if (lnw->pdev)
  //  pm_runtime_put(&lnw->pdev->dev);

  return 0;
}

static int lnw_gpio_direction_output(struct gpio_chip *chip,
                                     unsigned offset, int value)
{
  //struct lnw_gpio *lnw = to_lnw_priv(chip);
  void __iomem *gpdr = gpio_reg(chip, offset, GPDR);
  //unsigned long flags;

  lnw_gpio_set(chip, offset, value);

  //if (lnw->pdev)
  //  pm_runtime_get(&lnw->pdev->dev);

  //spin_lock_irqsave(&lnw->lock, flags);
  value = readl(gpdr);
  value |= BIT(offset % 32);
  writel(value, gpdr);
  //spin_unlock_irqrestore(&lnw->lock, flags);

  //if (lnw->pdev)
  //  pm_runtime_put(&lnw->pdev->dev);

  return 0;
}

void lnw_set_pininfo(struct lnw_gpio *lnw, unsigned gpio,
                     unsigned int type, const char *info)
{
  struct gpio_control *control;
  int num;
  DLOG("set_pininfo info = %s", info);
  DLOG("type = %u\n", type);

  control = find_gpio_control(lnw_gpio_controls,
                              ARRAY_SIZE(lnw_gpio_controls), type);
  if (control == NULL) {
    DLOG("%s: control == NULL", __FUNCTION__);
    return;
  }

  num = find_pininfo_num(control, info);
  if (num == -1) {
    DLOG("%s: num == -1", __FUNCTION__);
    return;
  }

  if (control->set)
    control->set(control, lnw, gpio, num);
}

static unsigned long __ffs(unsigned long word)
{
	int num = 0;
        
	if ((word & 0xffff) == 0) {
		num += 16;
		word >>= 16;
	}
	if ((word & 0xff) == 0) {
		num += 8;
		word >>= 8;
	}
	if ((word & 0xf) == 0) {
		num += 4;
		word >>= 4;
	}
	if ((word & 0x3) == 0) {
		num += 2;
		word >>= 2;
	}
	if ((word & 0x1) == 0)
		num += 1;
	return num;
}

static int lnw_set_maskunmask(u32 gpio, enum GPIO_REG reg_type,
				unsigned unmask)
{
	struct lnw_gpio *lnw = &lnw_gpio;
	unsigned long UNUSED flags;
	u32 value;
	void __iomem *gp_reg;

	gp_reg = gpio_reg(&lnw->chip, gpio, reg_type);

	spin_lock_irqsave(&lnw->lock, flags);

	if (unmask) {
		/* enable interrupt from GPIO input pin */
		value = readl(gp_reg) | BIT(gpio % 32);
	} else {
		/* disable interrupt from GPIO input pin */
		value = readl(gp_reg) & (~BIT(gpio % 32));
	}

	writel(value, gp_reg);

	spin_unlock_irqrestore(&lnw->lock, flags);

	return 0;
}


static void lnw_irq_mask(u32 gpio)
{
	struct lnw_gpio *lnw = &lnw_gpio;
	void __iomem UNUSED *gpit;

	if (gpio >= lnw->chip.ngpio)
		return;

	switch (lnw->type) {
	/* case CLOVERVIEW_GPIO_AON: */
	/* 	gpit = gpio_reg(&lnw->chip, gpio, GPIT); */

	/* 	/\* if it's level trigger, mask GPIM *\/ */
	/* 	if (readl(gpit) & BIT(gpio % 32)) */
	/* 		lnw_set_maskunmask(, GPIM, 0); */

	/* 	break; */
	case TANGIER_GPIO:
		lnw_set_maskunmask(gpio, GIMR, 0);
		break;
	default:
          panic("In default case in lnw_irq_mask");
		break;
	}
}


static uint32 lnw_irq_handler(uint8 vec)
{
  /* struct irq_data *data = irq_desc_get_irq_data(desc); */
  /* struct irq_chip *chip = irq_data_get_irq_chip(data); */
  struct lnw_gpio *lnw;
  struct gpio_debug UNUSED *debug;
  u32 base, gpio, mask;
  unsigned long pending;
  void __iomem *gp_reg;
  enum GPIO_REG reg_type;
  struct irq_desc UNUSED *lnw_irq_desc;
  unsigned int UNUSED lnw_irq;

  lnw = &lnw_gpio;

  //debug = lnw->debug;

  //DLOG("AAAAAAA");

  reg_type = (lnw->type == TANGIER_GPIO) ? GISR : GEDR;

  /* check GPIO controller to check which pin triggered the interrupt */
  for (base = 0; base < lnw->chip.ngpio; base += 32) {
    gp_reg = gpio_reg(&lnw->chip, base, reg_type);
    while ((pending = (lnw->type != TANGIER_GPIO) ?
            readl(gp_reg) :
            (readl(gp_reg) &
             readl(gpio_reg(&lnw->chip, base, GIMR))))) {
      gpio = __ffs(pending);
      //DLOG("gpio = %u\n", gpio);
      /* DEFINE_DEBUG_IRQ_CONUNT_INCREASE(lnw->chip.base + */
      /*                                  base + gpio); */
      /* Mask irq if not requested in kernel */
      /* lnw_irq = irq_find_mapping(lnw->domain, base + gpio); */
      /* lnw_irq_desc = irq_to_desc(lnw_irq); */
      /* if (lnw_irq_desc && unlikely(!lnw_irq_desc->action)) { */
      /*   lnw_irq_mask(&lnw_irq_desc->irq_data); */
      /*   continue; */
      /* } */

      mask = BIT(gpio);
      /* Clear before handling so we can't lose an edge */
      writel(mask, gp_reg);
      //generic_handle_irq(lnw_irq);
    }
    //DLOG("base = %u", base);
  }

  //chip->irq_eoi(data);
  return 0;
}


static bool
langwell_gpio_init(void)
{
  uint i, device_index, irq_pin;
  uint32 bar0, bar1;
  void *base;
  uint32 irq_base, gpio_base;
  lnw_gpio_t *lnw = &lnw_gpio;
  uint irq_line;
  pci_irq_t irq;

  if(scu_flis_probe() < 0) {
    panic("scu_flis_probe failed\n");
  }

  struct lnw_gpio_ddata_t *ddata = &lnw_gpio_ddata[TANGIER_GPIO];

  if (mp_ISA_PC) {
    DLOG ("Cannot operate without PCI");
    return FALSE;
  }

  /* Find the serial device on the PCI bus */
  device_index = ~0;
  i=0;
  while (pci_find_device (0x8086, 0x1199, 0x08, 0x80, i, &i)) {
    if (pci_get_device (i, &lnw_gpio.pdev)) {
      if (lnw_gpio.pdev.progIF == 0x00) {
        device_index =  i;
        break;
      }
      i++;
    }
    else {
      break;
    }
  }

  if (device_index == ~0) {
    DLOG ("Unable to find compatible device on PCI bus");
    return FALSE;
  }

  if (!pci_decode_bar (device_index, 0, &bar0, NULL, NULL)) {
    DLOG ("unable to decode BAR0");
    return FALSE;
  }

  if (!pci_decode_bar (device_index, 1, &bar1, NULL, NULL)) {
    DLOG ("unable to decode BAR1");
    return FALSE;
  }

  DLOG("bar0 = 0x%X", bar0);
  DLOG("bar1 = 0x%X", bar1);

  base = map_virtual_page((bar1 & 0xFFFFF000) | 0x1B);
  if(base == NULL) {
    panic("Failed to map langwell GPIO bar1");
  }
  base = (void*)(((uint32)base) | (0xFFF & bar1));
  irq_base = *(volatile u32 *)base;
  gpio_base = *((volatile u32 *)base + 1);

  unmap_virtual_page(base);

  DLOG("irq_base = 0x%X, gpio_base = 0x%X", irq_base, gpio_base);

  base = map_virtual_page((bar0 & 0xFFFFF000) | 0x1B);
  if(base == NULL) {
    panic("Failed to map langwell GPIO bar0");
  }
  base = (void*)(((uint32)base) | (0xFFF & bar0));

  lnw->type = TANGIER_GPIO;
  lnw->reg_base = base;
  lnw->reg_gplr = lnw->reg_base + ddata->gplr_offset;
  lnw->get_flis_offset = ddata->get_flis_offset;
  lnw->chip_irq_type = ddata->chip_irq_type;
  //lnw->chip.label = dev_name(&pdev->dev);
  lnw->chip.request = lnw_gpio_request;
  lnw->chip.direction_input = lnw_gpio_direction_input;
  lnw->chip.direction_output = lnw_gpio_direction_output;
  lnw->chip.set_pinmux = lnw_gpio_set_alt;
  lnw->chip.get_pinmux = gpio_get_alt;
  lnw->chip.get = lnw_gpio_get;
  lnw->chip.set = lnw_gpio_set;
  lnw->chip.to_irq = lnw_gpio_to_irq;
  lnw->chip.base = gpio_base;
  lnw->chip.ngpio = ddata->ngpio;
  lnw->chip.can_sleep = 0;
  lnw->chip.set_debounce = lnw_gpio_set_debounce;
  /* /\*lnw->domain = *\/ irq_domain_add_simple(pdev->dev.of_node, */
  /*                                          lnw->chip.ngpio, irq_base, */
  /*                                          &lnw_gpio_irq_ops, lnw); */

  for(i = 0; i < lnw->chip.ngpio; i++) {
    lnw_irq_mask(i);
  }

  lnw_irq_init_hw(lnw);

  if (!pci_get_interrupt (device_index, &irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    return FALSE;
  }

  DLOG ("Using IRQ pin=%X", irq_pin);

  /* if (pci_irq_find (lnw_gpio.pdev.bus, lnw_gpio.pdev.slot, irq_pin, &irq)) { */
  /*   /\* use PCI routing table *\/ */
  /*   DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi); */
  /*   if (!pci_irq_map_handler (&irq, lnw_irq_handler, 0x01, */
  /*                             IOAPIC_DESTINATION_LOGICAL, */
  /*                             IOAPIC_DELIVERY_FIXED)) { */
  /*     DLOG ("Unable to map IRQ handler"); */
  /*     return FALSE; */
  /*   } */
  /*   irq_line = irq.gsi; */
  /* } */
  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = langwell_gpio_init
};

DEF_MODULE (langwell_gpio, "Langwell GPIO drivre", &mod_ops, {"pci"});
