#ifndef BOARD_H
#define BOARD_H

#include "stm32l4xx.h"

void board_init(void);

void board_led_write(uint8_t state);
uint8_t board_button_read(void);

#endif
