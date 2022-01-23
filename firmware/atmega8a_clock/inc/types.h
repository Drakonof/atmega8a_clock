#ifndef INC_TYPES_H
#define INC_TYPES_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

typedef int32_t status_t;

typedef enum {
	error = -1,
	ok
} status_value;

#endif /* INC_TYPES_H */
