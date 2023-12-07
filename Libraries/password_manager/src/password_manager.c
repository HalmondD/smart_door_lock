#include "password_manager.h"

bool check_password(struct password* master_password, struct password* input_password)
{
	uint8_t password_index = 0;

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
	for (input_password->index = 0; input_password->index < input_password->byte_count; input_password->index++)
	{
		input_password->data_array[input_password->index] = 0;
	}

    input_password->index = 0;
}
