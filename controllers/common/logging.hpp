#ifndef LOGGING_HPP
#define LOGGING_HPP

#include <cstdarg>
#include <cstdio>


/**
 * @brief Simple logging function that wraps printf.
 *
 * @param prefix A string to print before the message (can be NULL).
 * @param fmt The format string for the message.
 * @param ... Arguments corresponding to the format string.
 */
static void log_msg(const char* prefix, const char* fmt, ...) {
    if (prefix) {
        printf("%s", prefix);
    }

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

#endif  // LOGGING_HPP
