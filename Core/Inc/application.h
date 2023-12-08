#ifndef INC_APP_DRIVER_H_
#define INC_APP_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "misc.h"
#include "keypad_driver.h"
#include "rfid_rc522_driver.h"
#include "lcd_1602_i2c_driver.h"

#define PASSWORD_COUNT  4

void app_loop(SPI_HandleTypeDef* hspi2, I2C_HandleTypeDef* hi2c1);

#endif /* INC_APP_DRIVER_H_ */
