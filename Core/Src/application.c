#include "application.h"

static void input_password_t(struct password* master_password, struct password* input_password);

void app_loop(SPI_HandleTypeDef* hspi2, I2C_HandleTypeDef* hi2c1)
{
	struct UID input_uid;
	rfid_rc522_init(hspi2);

	lcd_1602_init(hi2c1);

	uint8_t master_uid_data_array[4] = {0x00, 0x47, 0xE6, 0x1E};
	struct password master_uid;
	master_uid.data_array = master_uid_data_array;
	master_uid.count = 4;

	struct password checked_uid;
	checked_uid.data_array = input_uid.data_array;
	checked_uid.count = 4;

	uint8_t master_password_array[MASTER_PASSWORD_COUNT] = {'1', '2', '5', '6'};
	struct password master_password;
	master_password.data_array = master_password_array;
	master_password.count = MASTER_PASSWORD_COUNT;

	uint8_t input_password_array[MAX_INPUT_PASSWORD_COUNT];
	struct password input_password;
	input_password.data_array = input_password_array;
	input_password.count = 0;

	while (true)
	{
		while (rfid_rc522_wait_for_card() != true)
			gettick_delay_ms(300);
		
		rfid_rc522_read_serial(&input_uid);

		if (check_password(&master_uid, &checked_uid) == false)
		{
			lcd_1602_control_lcd_and_backlight(ENABLE);

			lcd_1602_print_full_screen("Wrong Card!", 1, 0, 2000);

			lcd_1602_control_lcd_and_backlight(DISABLE);

			break;
		}

		input_password_t(&master_password, &input_password);
		gettick_delay_ms(2000);
	}
}

static void input_password_t(struct password* master_password, struct password* input_password)
{
	uint8_t gia_tri_phim_nhan;
	bool	is_password_right = false;

	do
	{
		lcd_1602_control_lcd_and_backlight(ENABLE);

		lcd_1602_print_string("Input Password:", 1, 0);

		lcd_1602_set_cursor_position(2, 0);

		while ((gia_tri_phim_nhan = keypad_4x4_return_gia_tri_phim_nhan()) != '*')
		{
			input_password->count++;

			if (input_password->count > 15)
			{
				delete_password(input_password);

				lcd_1602_print_full_screen("Too Long!", 1, 0, 2000);
				lcd_1602_print_string("Input Password:", 1, 0);
				lcd_1602_set_cursor_position(2, 0);

				continue;
			}

			input_password->data_array[input_password->count - 1] = gia_tri_phim_nhan;

			lcd_1602_write_data('*');

			if (gia_tri_phim_nhan == '#')
			{
				delete_password(input_password);

				lcd_1602_clear_display();
				lcd_1602_print_string("Input Password:", 1, 0);
				lcd_1602_set_cursor_position(2, 0);
			}
		}

		if (check_password(master_password, input_password) == false)
		{
			delete_password(input_password);
			is_password_right = false;

			lcd_1602_print_full_screen("Wrong Password!", 1, 0, 2000);

			continue;
		}
		else
			is_password_right = true;
	}
	while (is_password_right == false);

	delete_password(input_password);
	lcd_1602_print_full_screen("Door Open", 1, 0, 2000);
	lcd_1602_control_lcd_and_backlight(DISABLE);
}

//static void welcome_lcd(struct UID* input_uid);
//static uint8_t turn_nibble_to_ascii(uint8_t nibble_data);

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
