#ifndef STRVAL_H
#define STRVAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>  // For bool
#include <string.h>   // For strlen, strcmp
#include <ctype.h>    // For isdigit, isalnum
#include <stdlib.h>   // For strtod, strtol
#include <errno.h>    // For errno
#include <limits.h>
#include <math.h>
#include "config.h"

bool is_alnum(const char *data);
bool str_is_bool(const char * data);
bool str_is_float(const char *str);
bool str_is_double(const char *str);
bool str_is_long(const char *str);
bool str_is_uint64(const char *str);
bool str_is_uint32(const char *str);
bool str_is_uint16(const char *str);
bool str_is_uint8(const char *str);
bool str_is_int64(const char *str);
bool str_is_int32(const char *str);
bool str_is_int16(const char *str);
bool str_is_int8(const char *str);

#ifdef __cplusplus
}
#endif

#endif