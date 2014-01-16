#ifndef ESC_H
#define ESC_H

#include <stdlib.h>
#include <inttypes.h>

// Hi-speed 500hz ESC implementation for exactly 4 motors
#define HZ 490 // must be <= 490

void initESCs();
void setESCs(uint16_t tl, uint16_t tr, uint16_t bl, uint16_t br); // values from 0 to 1000

#endif
