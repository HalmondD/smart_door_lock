#ifndef INC_PASSWORD_MANAGER_H_
#define INC_PASSWORD_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

struct password
{
    uint8_t byte_count;
    uint8_t *data_array;
    uint8_t index;
};

bool check_password(struct password* master_password, struct password* input_password);
void delete_password(struct password* input_password);
#endif /* INC_PASSWORD_MANAGER_H_ */