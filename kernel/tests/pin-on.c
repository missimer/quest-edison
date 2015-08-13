#include <gpio.h>
#include <stdio.h>

void delay(unsigned long ms)
{
    usleep(ms * 100);
}

#define DELAY 1

int main(void)
{
  gpio_direction_output(44, 0);

  while(1) {
    gpio_set_val(44, 1);
    delay(DELAY);
    gpio_set_val(44, 0);
    delay(DELAY);
  }
  return 0;
}
