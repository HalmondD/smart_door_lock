/*
 * keypad_driver.h
 *
 *  Created on: Oct 17, 2023
 *      Author: duong
 */

#ifndef INC_KEYPAD_DRIVER_H_
#define INC_KEYPAD_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#define KEYPAD_OUT_REGISTER  GPIOA->ODR
#define KEYPAD_IN_REGISTER   GPIOA->IDR

uint8_t keypad_4x4_return_gia_tri_phim_nhan(void);

#endif /* INC_KEYPAD_DRIVER_H_ */
