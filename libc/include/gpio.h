#ifndef _GPIO_H_
#define _GPIO_H_

#define GPIO_SYSCALL(_func, _ret, _pin, _arg1, _arg2)                         \
  do {                                                                        \
    asm volatile ("int $0x30\n" : "=a"(_ret): "a"(2L), "b"(_func), "c"(_pin), \
                  "d"(_arg1), "S"(_arg2) : "memory","cc","%edi");             \
  } while(0)

#define GPIO_DIRECTION_INPUT_FUNC  0
#define GPIO_DIRECTION_OUTPUT_FUNC 1
#define GPIO_SET_VAL_OUTPUT_FUNC   2
#define GPIO_SET_PININFO           3

static inline int
gpio_direction_input(unsigned int pin)
{
  int ret;

  GPIO_SYSCALL(GPIO_DIRECTION_INPUT_FUNC, ret, pin, 0, 0);

  return ret;
}

static inline int
gpio_direction_output(unsigned int pin, int outval)
{
  int ret;

  GPIO_SYSCALL(GPIO_DIRECTION_OUTPUT_FUNC, ret, pin, outval, 0);

  return ret;
}

static inline int
gpio_set_val(unsigned int pin, int val)
{
  int ret;

  GPIO_SYSCALL(GPIO_SET_VAL_OUTPUT_FUNC, ret, pin, val, 0);

  return ret;
}

static inline int
gpio_set_pininfo(unsigned int pin, unsigned int type, char* mode)
{
  int ret;

  GPIO_SYSCALL(GPIO_SET_PININFO, ret, pin, type, mode);

  return ret;
}

#endif // _GPIO_H_
