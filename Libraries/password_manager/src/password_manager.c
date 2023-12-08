#include "password_manager.h"

bool check_password(struct password* master_password, struct password* input_password)
{
	uint8_t password_index = 0;

	if (input_password->count != master_password->count)
		return false;

	

	for (password_index = 0; password_index <= 3; password_index++)
	{
		if (input_password->data_array[password_index] != master_password->data_array[password_index])
		{
			return false;
		}
	}

	return true;
}

void delete_password(struct password* input_password)
{
	uint8_t password_index;

	for (password_index = 0; password_index < input_password->count; password_index++)
	{
		input_password->data_array[password_index] = 0;
	}

    input_password->count = 0;
}
