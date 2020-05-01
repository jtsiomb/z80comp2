#include <stdio.h>
#include <string.h>
#include <stdarg.h>

int opt_loginstr;

void dbg_log_instr(const char *fmt, ...)
{
	va_list ap;

	if(!opt_loginstr) return;

	fprintf(stderr, "D: ");

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);

	fprintf(stderr, "\r\n");
}
