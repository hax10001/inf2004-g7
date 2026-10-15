#ifndef HOST_LIBBSP_H
#define HOST_LIBBSP_H
#include <tk/tkernel.h>
#define GPIO_MODE_IN 0U
ER gpio_set_pin(UINT pin, UINT mode);
UINT gpio_get_val(UINT pin);
ER gpio_set_val(UINT pin, UINT value);
ER pwm_set_pin(UINT pin);
ER pwm_set_wrap(UINT pin, UW wrap);
ER pwm_set_cc(UINT pin, UW compare);
ER pwm_set_enabled(UINT pin, BOOL enabled);
#endif
