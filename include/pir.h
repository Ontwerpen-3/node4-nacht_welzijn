#ifndef PIR_H
#define PIR_H

#include <stdint.h>

void pir_init(void);
uint8_t pir_detected(void);   // 1 = beweging, 0 = geen

#endif