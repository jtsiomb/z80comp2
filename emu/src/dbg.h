#ifndef DBG_H_
#define DBG_H_

#include <stdio.h>
#include <stdint.h>

extern int opt_loginstr;

void dbg_log_file(FILE *fp);
void dbg_log_pc(uint16_t pc);
void dbg_log_instr(const char *fmt, ...);

void dbg_mem_dump(uint16_t addr, int n);

#endif	/* DBG_H_ */
