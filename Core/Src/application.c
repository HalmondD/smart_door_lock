#include "application.h"

//static bool check_uid_data(struct UID* input_uid);
static void input_password_t(void);
static void delete_password_t(char* input_password_array, uint8_t* password_index, uint8_t* input_password_count);
/*
static void welcome_lcd(struct UID* input_uid);
static uint8_t turn_nibble_to_ascii(uint8_t nibble_data);
*/

void app_loop(SPI_HandleTypeDef* hspi2, I2C_HandleTypeDef* hi2c1)
{
	struct UID input_uid;
	rfid_rc522_init(hspi2);

	lcd_1602_i2c_init(hi2c1);

	uint8_t master_uid_data_array[4] = {0x00, 0x47, 0xE6, 0x1E};
	struct password master_uid;
	master_uid.byte_count = 4;
	master_uid.data_array = master_uid_data_array;
	master_uid.index = 0;

	struct password checked_uid;
	checked_uid.byte_count = 4;
	checked_uid.data_array = input_uid.data_array;
	checked_uid.index = 0;

	uint8_t master_password_array[PASSWORD_COUNT] = {'1', '2', '5', '6'};
	struct password master_password;
	master_password.byte_count = PASSWORD_COUNT;
	master_password.data_array = master_password_array;
	master_password.index = 0;

	uint8_t input_password_array[PASSWORD_COUNT];
	struct password input_password;
	input_password.byte_count = PASSWORD_COUNT;
	input_password.data_array = input_password_array;
	input_password.index = 0;

	while (true)
	{
		while (rfid_rc522_wait_for_card() != true)
			gettick_delay_ms(300);
		
		rfid_rc522_read_serial(&input_uid);

		if (check_password(&master_uid, &checked_uid) == false)
		{
			control_lcd_and_backlight(ENABLE);

			lcd_1602_i2c_set_cursor_position(1, 0);
			lcd_1602_i2c_print_string("Wrong Card!");
			gettick_delay_ms(2000);

			control_lcd_and_backlight(DISABLE);

			break;
		}

		input_password_t();
		gettick_delay_ms(2000);
	}
}

/*
static bool check_uid_data(struct UID* input_uid)
{
	uint8_t master_uid_data_array[4] = {0x00, 0x47, 0xE6, 0x1E};
	uint8_t uid_data_index = 0;

	for (uid_data_index = 0; uid_data_index <= 3; uid_data_index++)
	{
		if (input_uid->data_array[uid_data_index] != master_uid_data_array[uid_data_index])
		{
			control_lcd_and_backlight(ENABLE);

			lcd_1602_i2c_set_cursor_position(1, 0);
			lcd_1602_i2c_print_string("Wrong Card!");
			gettick_delay_ms(2000);

			control_lcd_and_backlight(DISABLE);
			return false;
		}
	}

	return true;
}
*/

static void input_password_t(void)
{
	uint8_t gia_tri_phim_nhan;
	char	input_password_array[PASSWORD_COUNT];
	char	master_password_array[PASSWORD_COUNT] = {"1256"};
	uint8_t password_index = 0;
	uint8_t	input_password_count = 0;
	bool	is_password_right = false;

	do
	{
		control_lcd_and_backlight(ENABLE);

		lcd_1602_i2c_set_cursor_position(1, 0);
		lcd_1602_i2c_print_string("Input Password:");

		lcd_1602_i2c_set_cursor_position(2, 0);

		while ((gia_tri_phim_nhan = keypad_4x4_return_gia_tri_phim_nhan()) != '*')
		{
			input_password_count++;

			if (input_password_count > 15)
			{
				delete_password_t(input_password_array, &password_index, &input_password_count);

				clear_display_and_print("Too Long!", 2000);
				clear_display_and_print("Input Password:", 0);
				lcd_1602_i2c_set_cursor_position(2, 0);

				continue;
			}

			input_password_array[password_index] = gia_tri_phim_nhan;
			password_index++;

			lcd_1602_i2c_write_data('*');

			if (gia_tri_phim_nhan == '#')
			{
				delete_password_t(input_password_array, &password_index, &input_password_count);

				clear_display_and_print("Input Password:", 0);
				lcd_1602_i2c_set_cursor_position(2, 0);
			}
		}

		for (password_index = 0; password_index <= PASSWORD_COUNT - 1; password_index++)
		{
			if ((master_password_array[password_index] != input_password_array[password_index]) | (input_password_count > PASSWORD_COUNT))
			{
				delete_password_t(input_password_array, &password_index, &input_password_count);
				is_password_right = false;

				clear_display_and_print("Wrong Password!", 2000);

				break;
			}
			else
				is_password_right = true;
		}
	}
	while (is_password_right == false);

	clear_display_and_print("Door Open", 2000);
	control_lcd_and_backlight(DISABLE);
}

static void delete_password_t(char* input_password_array, uint8_t* password_index, uint8_t* input_password_count)
{
	*password_index = 0;
	*input_password_count = 0;
	for (int i = 0; i < PASSWORD_COUNT; i++)
	{
		input_password_array[i] = 0;
	}
}

/*
static void welcome_lcd(struct UID* input_uid)
{
	char lcd_uid_data[12] = {"00 00 00 00"};
	uint8_t lcd_uid_data_index;
	uint8_t high_nibble, low_nibble;

	lcd_1602_i2c_write_instruction(CLEAR_DISPLAY);
	gettick_delay_ms(2);
	lcd_1602_i2c_control_backlight(BACKLIGHT_ON);
	lcd_1602_i2c_write_instruction(LCD_ON);
	gettick_delay_ms(1);

	lcd_1602_i2c_set_cursor_position(1, 0);

	uint8_t uid_data_index;

	for (uid_data_index = 0, lcd_uid_data_index = 0; uid_data_index <= 3; uid_data_index++, lcd_uid_data_index += 2)
		{
			high_nibble = (input_uid->data_array[uid_data_index] >> 4) & 0x0F;
			low_nibble	= input_uid->data_array[uid_data_index] & 0x0F;

			lcd_uid_data[lcd_uid_data_index] = turn_nibble_to_ascii(high_nibble);
			lcd_uid_data[++lcd_uid_data_index] = turn_nibble_to_ascii(low_nibble);
		}

	lcd_1602_i2c_write_instruction(CLEAR_DISPLAY);
	gettick_delay_ms(2);

	lcd_1602_i2c_set_cursor_position(1, 0);
	lcd_1602_i2c_print_string("Hello, uid:");

	lcd_1602_i2c_set_cursor_position(2, 0);
	lcd_1602_i2c_print_string(lcd_uid_data);

	gettick_delay_ms(2000);
}

static uint8_t turn_nibble_to_ascii(uint8_t nibble_data)
{
	if (nibble_data < 10)
		return '0' + nibble_data;
	else
		return 'A' + (nibble_data - 10);
}
*/
