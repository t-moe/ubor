#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <stdarg.h>

uint8_t display_log(uint8_t id, const char* fmtstr, ...);
void display_init();

#define DISPLAY_NEWLINE 0

#endif /* DISPLAY_H */
