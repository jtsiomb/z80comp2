#ifndef EMU_H_
#define EMU_H_

#include <stdint.h>

int emu_init(void *romimg, int romsz);
void emu_cleanup(void);

void emu_reset(void);
void emu_step(void);

uint8_t emu_mem_read(uint16_t addr);
void emu_mem_write(uint16_t addr, uint8_t data);
uint8_t emu_io_read(uint16_t addr);
void emu_io_write(uint16_t addr, uint8_t data);

void emu_serin(int port, int c);
void emu_serout(int port, int c);	/* implemented by the backend */

#endif	/* EMU_H_ */
