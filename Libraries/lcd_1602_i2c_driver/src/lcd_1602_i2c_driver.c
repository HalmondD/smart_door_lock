#include "lcd_1602_i2c_driver.h"

I2C_HandleTypeDef* ptr_lcd_1602_i2c_connect_mode;
uint8_t backlight_state = BACKLIGHT_OFF;

static void lcd_1602_i2c_write_4bit(uint8_t write_data);
static void lcd_1602_i2c_pulse_enable(uint8_t write_data);

/**
 * The delay before initialization will not help in the long run.
 * 
 * The state of IO expander controlling the LCD bus and the LCD controller are reset to known state
 * only when there is a power-up, as they have a power-up reset.
 * 
 * After power-up reset, the LCD bus (IO expander) will be set to all pins high, and the LCD controller is in 8-bit mode,
 * waiting for 8-bit data cycles. The current initialization is not perfect and fails to go reliably into 4-bit mode.
 * The first falling E pulse of first nibble is understood as 8-bit command, and after it the LCD goes to 4-bit mode,
 * but the sending of second nibble throws the bus out of sync, as the LCD is expecting first nibble of 4-bit command now.
 * So in fact, after power-up, the sequence will always cause the display nybbles to be out of sync, by a nybble.
 * That is why resetting MCU helps, as the sequence might get back to sync accidentally.
 * 
 * But when MCU code has been running, and if only MCU is reset,
 * there is no guarantee at which state the LCD bus (IO expander output) or the LCD controller is,
 * so that sequence should be hand-crafted to walk out of any state to a known state.
 * As there is no reset pins or ability to power-cycle the IO expander and LCD via IO pin, it must be performed via I2C bus.
 * 
 * The LCD Enable pin might already be up, while RS and RW need to be set up before rising edge of Enable.
 * Therefore the IO expander (LCD bus) needs one I2C write to get it to known state, preferably by setting Enable low if it was high,
 * and backlight state set to whatever necessary. If it was high, this can execute an unwanted LCD command or send data to it, but not to worry, it is handled next.
 * 
 * The LCD is most likely in 4-bit mode from previous running sequence of MCU, but it is unknown whether it is expecting the first nibble or second nibble of 4-bit command,
 * or still in 8-bit mode after power-up reset. Therefore, the proper sequence to syncronize the 4-bit bus mode is to send single nybbles to LCD at first,
 * commanding it to go back to 8-bit mode. It needs three nybbles of commands to go to 8-bit mode, but there must be enough time between the nybbles,
 * to allow execution of the slowest commands available after the first and second nybble. When the LCD is properly set back to 8-bit mode,
 * it can be commanded to go back to 4-bit mode. This needs basically three nybbles - first an 8-bit command to go to 4-bit mode,
 * and then it will be in 4-bit mode, but without proper font and lines configuration. From this point on, the command bytes can be sent as two nybbles as usual,
 * so first thing it needs is the command byte again as two nybbles, so it stays in 4-bit mode but this time it gets proper font and lines config. 
 */
void lcd_1602_i2c_init(I2C_HandleTypeDef* connect_mode)
{
    ptr_lcd_1602_i2c_connect_mode = connect_mode;
    
    //reset khoi dong nguon
    //delay 100ms sau khi cap nguon
    gettick_delay_ms(55);
    // Reset LCD_ENABLE
    lcd_1602_i2c_write_4bit(0x00);
    // Turn backlight off
    backlight_state = BACKLIGHT_OFF;
    //delay 4.5 ms sau khi gui lenh 0x30 lan 1
    lcd_1602_i2c_write_4bit(RESET_INIT_1);
    lcd_1602_i2c_pulse_enable(RESET_INIT_1);
    gettick_delay_ms(5);
    //delay 100us sau khi gui lenh 0x30 lan 2
    lcd_1602_i2c_write_4bit(RESET_INIT_1);
    lcd_1602_i2c_pulse_enable(RESET_INIT_1);
    gettick_delay_ms(5);
    //delay 100us sau khi gui lenh 0x30 lan 3
    lcd_1602_i2c_write_4bit(RESET_INIT_1);
    lcd_1602_i2c_pulse_enable(RESET_INIT_1);
    gettick_delay_ms(5);
    //delay 100us sau khi gui lenh 0x20
    lcd_1602_i2c_write_4bit(RESET_INIT_2);
    lcd_1602_i2c_pulse_enable(RESET_INIT_2);
    gettick_delay_ms(1);

    //khoi dong LCD
    //delay 100us sau khi gui lenh function set
    lcd_1602_i2c_write_instruction(FUNCTION_SET);
    gettick_delay_ms(1);
    //delay 2 ms sau khi gui lenh clear display
    lcd_1602_i2c_write_instruction(CLEAR_DISPLAY);
    gettick_delay_ms(2);
    //delay 2ms sau khi gui lenh display control
    lcd_1602_i2c_write_instruction(LCD_OFF);
    gettick_delay_ms(1);
    //delay 100us sau khi gui lenh entry mode set
    lcd_1602_i2c_write_instruction(ENTRY_MODE_SET);
    gettick_delay_ms(1);
}

void lcd_1602_i2c_write_instruction(uint8_t lcd_instruction)
{   
    uint8_t high_nibble = (lcd_instruction & 0xF0) | WRITE_INSTRUCTION;
    uint8_t low_nibble  = ((lcd_instruction << 4) & 0xF0) | WRITE_INSTRUCTION;

    lcd_1602_i2c_write_4bit(high_nibble);
    lcd_1602_i2c_pulse_enable(high_nibble);

    lcd_1602_i2c_write_4bit(low_nibble);
    lcd_1602_i2c_pulse_enable(low_nibble);
}

void lcd_1602_i2c_write_data(uint8_t lcd_data)
{   
    uint8_t high_nibble = (lcd_data & 0xF0) | WRITE_DATA;
    uint8_t low_nibble  = ((lcd_data << 4) & 0xF0) | WRITE_DATA;

    lcd_1602_i2c_write_4bit(high_nibble);
    lcd_1602_i2c_pulse_enable(high_nibble);

    lcd_1602_i2c_write_4bit(low_nibble);
    lcd_1602_i2c_pulse_enable(low_nibble);
}

void lcd_1602_i2c_print_string(const char string_array[])
{
    while (*string_array != END_OF_LINE)
    {
        lcd_1602_i2c_write_data(*string_array);
        string_array++;
    }       
}

static void lcd_1602_i2c_write_4bit(uint8_t write_data)
{
    uint8_t data = write_data | backlight_state;
    HAL_I2C_Master_Transmit(ptr_lcd_1602_i2c_connect_mode, DEVICE_I2C_ADDRESS, (uint8_t*)&data, 1, 1);
}

static void lcd_1602_i2c_pulse_enable(uint8_t write_data)
{
    lcd_1602_i2c_write_4bit(write_data | LCD_ENABLE);
    gettick_delay_ms(1);

    lcd_1602_i2c_write_4bit(write_data & (~LCD_ENABLE));
    gettick_delay_ms(1);
}

void lcd_1602_i2c_set_cursor_position(uint8_t vi_tri_cot, uint8_t vi_tri_hang)
{
    uint8_t cursor_address_array[3] = {0x0, 0x80, 0xC0};
    uint8_t cursor_address = cursor_address_array[vi_tri_cot] + vi_tri_hang;
    lcd_1602_i2c_write_instruction(cursor_address);
    gettick_delay_ms(1);
}

void lcd_1602_i2c_control_backlight(uint8_t backlight_data)
{
    backlight_state = backlight_data;
}

void control_lcd_and_backlight(bool control_state)
{
	lcd_1602_i2c_write_instruction(CLEAR_DISPLAY);
	gettick_delay_ms(2);

	if (control_state == ENABLE)
	{
		lcd_1602_i2c_control_backlight(BACKLIGHT_ON);
		lcd_1602_i2c_write_instruction(LCD_ON);
		gettick_delay_ms(1);
	}
	else
	{
		lcd_1602_i2c_control_backlight(BACKLIGHT_OFF);
		lcd_1602_i2c_write_instruction(LCD_OFF);
		gettick_delay_ms(1);
	}
}

void clear_display_and_print(const char string_array[], uint16_t delay_time_ms)
{
	lcd_1602_i2c_write_instruction(CLEAR_DISPLAY);
	gettick_delay_ms(2);
	lcd_1602_i2c_set_cursor_position(1, 0);

	lcd_1602_i2c_print_string(string_array);

	if (delay_time_ms != 0)
		gettick_delay_ms(delay_time_ms);
}