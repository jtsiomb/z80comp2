#include <stdlib.h>
#include "emu.h"
#include "cpu.h"
#include "duart.h"

#define RAM_SIZE	0x8000
static uint8_t ram[RAM_SIZE];
static uint8_t *rom;
static int rom_size;

int emu_init(void *romimg, int romsz)
{
	rom = romimg;
	rom_size = romsz;

	emu_reset();
	return 0;
}

void emu_reset(void)
{
	cpu_reset();
	duart_reset();
}

void emu_cleanup(void)
{
}

void emu_step(void)
{
	cpu_step();
}

uint8_t emu_mem_read(uint16_t addr)
{
	if(addr & 0x8000) {	/* RAM */
		return ram[addr & 0x7fff];
	} else { /* ROM */
		if((int)addr < rom_size) {
			return rom[addr];
		}
	}
	return 0;
}

void emu_mem_write(uint16_t addr, uint8_t data)
{
	if(addr & 0x8000) {
		ram[addr & 0x7fff] = data;
	}
}

uint8_t emu_io_read(uint16_t addr)
{
	if((addr & 0xf0) == 0) {
		return duart_read(addr & 0xf);
	}

	/* nothing is enabled, the bus floats, return random bits */
	return rand();
}

void emu_io_write(uint16_t addr, uint8_t data)
{
	if((addr & 0xf0) == 0) {
		duart_write(addr & 0xf, data);
	}
}

void emu_serin(int port, int c)
{
	duart_serin(port, c);
}
