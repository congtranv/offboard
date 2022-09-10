#include "offboard/ivsr_log.h"
#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include <string>

void ivsrLogInfo(const char* format, ...) {
    char msg[100];
    va_list arg_ptr;
    va_start(arg_ptr, format);
    vsprintf(msg, format, arg_ptr);
    va_end(arg_ptr);
    std::printf("[ INFO] %s \n", msg);
}

void ivsrLogWarn(const char* format, ...) {
    char msg[100];
    va_list arg_ptr;
    va_start(arg_ptr, format);
    vsprintf(msg, format, arg_ptr);
    va_end(arg_ptr);
    std::printf("[ WARN] %s \n", msg);
}

void ivsrLogDebug(const char* format, ...) {
    char msg[100];
    va_list arg_ptr;
    va_start(arg_ptr, format);
    vsprintf(msg, format, arg_ptr);
    va_end(arg_ptr);
    std::printf("[ DEBUG] %s \n", msg);
}

void ivsrLogError(const char* format, ...) {
    char msg[100];
    va_list arg_ptr;
    va_start(arg_ptr, format);
    vsprintf(msg, format, arg_ptr);
    va_end(arg_ptr);
    std::printf("[ ERROR] %s \n", msg);
}
