#ifndef INC_APP_DRIVER_H_
#define INC_APP_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "misc.h"
#include "password_manager.h"
#include "keypad_driver.h"
#include "rfid_rc522_driver.h"
#include "lcd_1602_driver.h"

#define MASTER_PASSWORD_COUNT       4
#define MAX_INPUT_PASSWORD_COUNT    16

void app_loop(SPI_HandleTypeDef* hspi2, I2C_HandleTypeDef* hi2c1);

#endif /* INC_APP_DRIVER_H_ */
