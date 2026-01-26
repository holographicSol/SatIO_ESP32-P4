#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <math.h>
#include "dtoscinot.h"

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
char* doubleToScientific(double value, int maxChars, char *buffer) {
    if (maxChars <= 1) {
        buffer[0] = '\0';
        return buffer;
    }
    // Handle specials first (short-circuit cheap cases)
    if (isnan(value)) {
        strncpy(buffer, "NAN", maxChars);
        buffer[maxChars - 1] = '\0';
        return buffer;
    }
    if (isinf(value)) {
        if (value < 0) strncpy(buffer, "-INF", maxChars);
        else         strncpy(buffer, "INF",  maxChars);
        buffer[maxChars - 1] = '\0';
        return buffer;
    }
    if (maxChars < 8) {  // Can't fit even "-9.e+99\0"
        strncpy(buffer, "OMAX", maxChars);
        buffer[maxChars - 1] = '\0';
        return buffer;
    }
    // Estimate room for digits after decimal point
    int avail = maxChars - 6;           // base: sign + digit + '.' + 'e' + sign + 2-digit-exp
    if (fabs(value) >= 1e100) avail -= 1; // might need 3-digit exponent
    if (value < 0) avail--;               // extra sign

    int prec = avail - 1;               // prec = digits after decimal
    if (prec < 0) prec = 0;
    if (prec > 15) prec = 15;           // double realistically has ~15 decimal digits

    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%.%de", prec);  // "%.Ne"

    int written = snprintf(buffer, maxChars, fmt, value);

    // If it didn't fit or looks wrong → reduce precision and retry
    if (written < 0 || written >= maxChars || !strchr(buffer, 'e')) {
        // Fall back: try with fewer digits
        for (int p = prec - 1; p >= 0; p--) {
            snprintf(fmt, sizeof(fmt), "%%.%de", p);
            written = snprintf(buffer, maxChars, fmt, value);
            if (written > 0 && written < maxChars && strchr(buffer, 'e')) {
                return buffer;  // success
            }
        }
        // Still no luck → minimal
        snprintf(buffer, maxChars, "%.0e", value);
    }
    // Last resort if still broken
    if (strlen(buffer) >= (size_t)maxChars || !strchr(buffer, 'e')) {
        strncpy(buffer, "OMAX", maxChars);
        buffer[maxChars - 1] = '\0';
    }
    return buffer;
}