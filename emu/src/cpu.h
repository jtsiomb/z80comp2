#ifndef CPU_H_
#define CPU_H_

enum {
	FLAGS_C		= 0x01,
	FLAGS_N		= 0x02,
	FLAGS_PV	= 0x04,
	FLAGS_H		= 0x10,
	FLAGS_Z		= 0x40,
	FLAGS_S		= 0x80
};

struct regs8 {
	uint8_t f, a, c, b, e, d, l, h;
};
struct regs16 {
	uint16_t af, bc, de, hl;
};

union genregs {
	struct regs8 r;
	struct regs16 rr;
};

struct registers {
	union genregs g, shadow;	/* general purpose and shadow registers */
	uint8_t i, r;
	uint16_t ix, iy, sp, pc;
	uint8_t iff, imode;
};

void cpu_reset(void);
void cpu_step(void);
void cpu_intr(void);
void cpu_breakpt(void);
struct registers *cpu_regs(void);
int cpu_get_halt(void);
int cpu_get_intr(void);

int cpu_set_named(const char *name, unsigned int val);
int cpu_get_named(const char *name);

#endif	/* CPU_H_ */
