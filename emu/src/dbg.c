#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include "dbg.h"
#include "emu.h"

int opt_loginstr;

static uint16_t cur_addr;
static FILE *out;

void dbg_log_file(FILE *fp)
{
	out = fp;
}

void dbg_log_pc(uint16_t pc)
{
	cur_addr = pc;
}

void dbg_log_instr(const char *fmt, ...)
{
	va_list ap;

	if(!opt_loginstr || !out) return;

	fprintf(out, "%04x: ", cur_addr);

	va_start(ap, fmt);
	vfprintf(out, fmt, ap);
	va_end(ap);

	fprintf(out, "\r\n");
}

#define ROW_BYTES	16
void dbg_mem_dump(uint16_t addr, int n)
{
	int i;
	uint16_t end = addr + n;

	while(n > 0) {
		printf("%04x:", (unsigned int)addr);
		for(i=0; i<ROW_BYTES; i++) {
			if((i & 7) == 0) putchar(' ');
			if(addr + i < end) {
				printf(" %02x", (unsigned int)emu_mem_read(addr + i));
			} else {
				fputs("   ", stdout);
			}
		}
		fputs("  |", stdout);
		for(i=0; i<ROW_BYTES; i++) {
			uint8_t val = emu_mem_read(addr + i);
			char c = *(char*)&val;
			putchar(isprint(c) ? c : '.');
		}
		fputs("|\n", stdout);
		n -= ROW_BYTES;
		addr += ROW_BYTES;
	}
}
