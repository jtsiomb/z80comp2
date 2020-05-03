#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "dbg.h"

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
