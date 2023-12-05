#include "state_machine.h"

static bool check_uid_data(struct UID* input_uid);

void idle_state()
{
	/// ENTRY ///
	/// DO ///
	/// EXIT ///
	;
}

void input_key_state()
{
	/// ENTRY ///
	/// DO ///
    while (true)
	{
		while (rfid_rc522_wait_for_card() != true)
			gettick_delay_ms(300);
		
		rfid_rc522_read_serial(&input_uid);

		if (check_uid_data(&input_uid) == false)
			break;

		input_password_state();
		gettick_delay_ms(2000);
	}

	/// EXIT ///
}

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

void forget_key_state()
{
	/// ENTRY ///
	/// DO ///
	/// EXIT ///
	;
}

void burglar_state()
{
	/// ENTRY ///
	/// DO ///
	/// EXIT ///
	;
}

void leave_house_state()
{
	/// ENTRY ///
	/// DO ///
	/// EXIT ///
	;
}

void reset_key_state()
{
	/// ENTRY ///
	/// DO ///
	/// EXIT ///
	;
}