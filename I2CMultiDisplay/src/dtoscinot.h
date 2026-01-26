#ifndef DTOSCINOT_H
#define DTOSCINOT_H

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <math.h>

/**
 * @brief doubleToScientific - Converts a double to scientific notation string.
 * Allows very large numbers to be represented on small displays / confined spaces
 * or as scientific notation is otherwise required.
 * 
 * @param value The double value to convert.
 * @param maxChars The maximum number of characters in the output buffer.
 * @param buffer The output buffer to store the resulting string.
 * @return Pointer to the output buffer.
 * 
 * @note Can be used in cases where dtostre() is not available.
 * 
 * @warning Minimum maxChars is 8.
 */
char* doubleToScientific(double value, int maxChars, char *buffer);

#endif // DTOSCINOT_H