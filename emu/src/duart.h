#ifndef DUART_H_
#define DUART_H_

#include <stdint.h>

void duart_reset(void);
void duart_serin(int port, int c);

uint8_t duart_read(int rs);
void duart_write(int rs, uint8_t data);

#endif	/* DUART_H_ */
