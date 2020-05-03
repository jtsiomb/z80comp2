#ifndef DBG_H_
#define DBG_H_

#include <stdio.h>
#include <stdint.h>

extern int opt_loginstr;

void dbg_log_file(FILE *fp);
int dbg_begin_instr(uint16_t pc);
void dbg_end_instr(void);
void dbg_log_instr(const char *fmt, ...);

void dbg_mem_dump(uint16_t addr, int n);

void dbg_setbpt(uint16_t addr);
int dbg_delbpt(uint16_t addr);

#endif	/* DBG_H_ */
