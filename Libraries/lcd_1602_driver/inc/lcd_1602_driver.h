/*
 * lcd_1602_i2c_driver.h
 *
 *  Created on: Nov 11, 2023
 *      Author: duong
 */

#ifndef INC_LCD_1602_I2C_DRIVER_H_
#define INC_LCD_1602_I2C_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "misc.h"

#define DEVICE_I2C_ADDRESS      (0X27 << 1)

#define WRITE_INSTRUCTION       0X00
#define WRITE_DATA              0X01

#define LCD_ENABLE              0X04

#define RESET_INIT_1            0X30
#define RESET_INIT_2            0X20
#define FUNCTION_SET            0X28
#define ENTRY_MODE_SET          0X06

#define CLEAR_DISPLAY           0X01

#define ONE_LINE_MODE           
#define TWO_LINE_MODE           0x38

#define LCD_ON                  0X0C
#define LCD_OFF                 0X08

#define BACKLIGHT_ON            0X08
#define BACKLIGHT_OFF           0X00

#define END_OF_LINE             0X00

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////
void lcd_1602_write_instruction(uint8_t lcd_instruction);
void lcd_1602_write_data(uint8_t lcd_data);

/////////////////////////////////////////////////////////////////////////////////////
// Functions for init/setup the LCD 1602
/////////////////////////////////////////////////////////////////////////////////////
void lcd_1602_init(I2C_HandleTypeDef* connect_mode);
void lcd_1602_set_cursor_position(uint8_t vi_tri_cot, uint8_t vi_tri_hang);
void lcd_1602_control_backlight(uint8_t backlight_data);
void lcd_1602_control_lcd_and_backlight(bool control_state);

/////////////////////////////////////////////////////////////////////////////////////
// Application functions for the LCD 1602
/////////////////////////////////////////////////////////////////////////////////////
void lcd_1602_clear_display();
void lcd_1602_wait_and_clear(uint16_t delay_time_ms);
void lcd_1602_print_string(const char string_array[], uint8_t vi_tri_cot, uint8_t vi_tri_hang);
void lcd_1602_print_full_screen(const char string_array[], uint8_t vi_tri_cot, uint8_t vi_tri_hang, uint16_t delay_time_ms);

#endif /* INC_LCD_1602_I2C_DRIVER_H_ */
