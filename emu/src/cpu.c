#include <string.h>
#include <stdint.h>
#include "cpu.h"
#include "emu.h"
#include "dbg.h"

enum {
	R_B	= 0,
	R_C	= 1,
	R_D	= 2,
	R_E	= 3,
	R_H	= 4,
	R_L	= 5,
	R_INVAL = 6,
	R_A	= 7
};

static const char *r8str[] = {"b", "c", "d", "e", "h", "l", "(hl)", "a", 0};

#define RRSET2		0x4
#define RRSET_IX	0x8
#define RRSET_IY	0xc
enum {
	RR_BC = 0,
	RR_DE = 1,
	RR_HL = 2,
	RR_SP = 3,

	RR2_BC = RRSET2 | RR_BC,
	RR2_DE = RRSET2 | RR_DE,
	RR2_HL = RRSET2 | RR_HL,
	RR2_AF = RRSET2 | RR_SP,

	RRX_BC = RRSET_IX | RR_BC,
	RRX_DE = RRSET_IX | RR_DE,
	RRX_IX = RRSET_IX | RR_HL,
	RRX_SP = RRSET_IX | RR_SP,

	RRY_BC = RRSET_IY | RR_BC,
	RRY_DE = RRSET_IY | RR_DE,
	RRY_IY = RRSET_IY | RR_HL,
	RRY_SP = RRSET_IY | RR_SP
};

static const char *r16str[] = {
	"bc", "de", "hl", "sp",		/* default set */
	"bc", "de", "hl", "af",		/* RRSET2 */
	"bc", "de", "ix", "sp",		/* RRSET_IX */
	"bc", "de", "iy", "sp",		/* RRSET_IY */
	0
};

enum {
	PREFIX_ED	= 1,
	PREFIX_CB	= 2,
	PREFIX_DD	= 4,
	PREFIX_FD	= 8
};

enum {
	ALUOP_ADD	= 0,
	ALUOP_ADC	= 1,
	ALUOP_SUB	= 2,
	ALUOP_SBC	= 3,
	ALUOP_AND	= 4,
	ALUOP_XOR	= 5,
	ALUOP_OR	= 6,
	ALUOP_CP	= 7
};

static const char *aluopstr[] = {
	"add a,", "adc a,", "sub", "sbc a,", "and", "xor", "or", "cp"
};

enum {
	CC_NZ	= 0,
	CC_Z	= 1,
	CC_NC	= 2,
	CC_C	= 3,
	CC_PO	= 4,
	CC_PE	= 5,
	CC_P	= 6,
	CC_M	= 7
};

static const char *ccstr[] = {"nz", "z", "nc", "c", "po", "pe", "p", "m"};

enum {
	SR_SIGN_EXT		= 1,
	SR_SETFL		= 2,
	SR_ROT_CARRY	= 4,
	SR_ROT_NOCARRY	= 8
};

enum {
	LDI_ONCE	= 0,
	LDI_REP		= 1
};

static void runop_main(uint8_t op);
static void runop_ed(uint8_t op);
static void runop_cb(uint8_t op);
static void runop_dd(uint8_t op);
static void runop_fd(uint8_t op);
static void runop_ddcb(uint8_t op);
static void runop_fdcb(uint8_t op);

static void set_reg8(int r, uint8_t val);
static void set_reg8s(int r, int8_t val);
static uint8_t get_reg8(int r);
static int8_t get_reg8s(int r);
static void set_reg16(int r, uint16_t val);
static void set_reg16s(int r, int16_t val);
static uint16_t get_reg16(int r);
static int16_t get_reg16s(int r);
static void set_flag(unsigned int flag, int val);
static int parity(int x);
static int overflow(int x);
static int overflow16(int x);


static void op_load_reg8_reg8(int rdest, int rsrc);
static void op_load_reg8_imm8(int rdest, uint8_t imm);
static void op_load_reg8_mem(int rdest, uint16_t addr);
static void op_store_mem_reg8(uint16_t addr, int rsrc);
static void op_store_mem_imm8(uint16_t addr, uint8_t imm);
static void op_store_mem_reg16(uint16_t addr, int rsrc);
static void op_load_reg16_imm16(int rdest, uint16_t imm);
static void op_load_reg16_reg16(int rdest, int rsrc);
static void op_load_reg16_mem(int rdest, uint16_t addr);
static void op_exch_mem_reg16(uint16_t addr, int rr);
static void op_alu_reg8(int op, int r);
static void op_alu_mem(int op, uint16_t addr);
static void op_alu_imm8(int op, uint8_t imm);
static void op_incdec_reg8(int r, int adj);
static void op_incdec_reg16(int r, int adj);
static void op_incdec_mem(uint16_t addr, int adj);
static void op_add_reg16_reg16(int rdest, int rsrc);
static void op_adc_sbc_reg16_reg16(int rdest, int rsrc, int sub);
static void op_push_reg16(int r);
static void op_pop_reg16(int r);
static void op_call(uint16_t addr);
static void op_ret(void);
static void op_input(int r, uint16_t addr);
static void op_output(uint16_t addr, int r);

static void push(uint16_t val);
static uint16_t pop(void);

static uint8_t sr_left(uint8_t x, unsigned int mode);
static uint8_t sr_right(uint8_t x, unsigned int mode);

static void op_load_incdec(int step, unsigned int mode);
static void op_cmp_incdec(int step, unsigned int mode);
static void op_input_incdec(int step, unsigned int mode);
static void op_output_incdec(int step, unsigned int mode);

static struct registers regs;
static int halt;
static int prev_instr_ei;

static void (*runop[16])(uint8_t op) = {
	runop_main,		/* 0000: no prefix */
	runop_ed,		/* 0001: ED prefix */
	runop_cb,		/* 0010: CB prefix */
	0,				/* 0011: CBED invalid */
	runop_dd,		/* 0100: DD prefix */
	0,				/* 0101: DDED invalid */
	runop_ddcb,		/* 0110: DDCB prefix */
	0,				/* 0111: DDCBED invalid */
	runop_fd,		/* 1000: FD prefix */
	0,				/* 1001: FDED invalid */
	runop_fdcb,		/* 1010: FDCB prefix */
	0, 0, 0, 0, 0	/* all the rest combinations are invalid */
};

static uint8_t zero8;

static uint8_t *regptr8[8] = {
	&regs.g.r.b, &regs.g.r.c,
	&regs.g.r.d, &regs.g.r.e,
	&regs.g.r.h, &regs.g.r.l,
	&zero8, &regs.g.r.a
};

static uint16_t *regptr16[] = {
	&regs.g.rr.bc, &regs.g.rr.de, &regs.g.rr.hl, &regs.sp,
	&regs.g.rr.bc, &regs.g.rr.de, &regs.g.rr.hl, &regs.g.rr.af,
	&regs.g.rr.bc, &regs.g.rr.de, &regs.ix, &regs.sp,
	&regs.g.rr.bc, &regs.g.rr.de, &regs.iy, &regs.sp
};

void cpu_reset(void)
{
	regs.iff = 0;
	regs.pc = 0;
	regs.i = regs.r = 0;
	regs.imode = 0;
}

static uint8_t fetch_byte(void)
{
	return emu_mem_read(regs.pc++);
}

static int8_t fetch_sbyte(void)
{
	uint8_t byte = fetch_byte();
	return *(int8_t*)&byte;
}

static uint16_t fetch_imm16(void)
{
	uint16_t lsb = emu_mem_read(regs.pc++);
	uint16_t msb = emu_mem_read(regs.pc++);
	return lsb | (msb << 8);
}

static unsigned int prefix_bit(uint8_t op)
{
	switch(op) {
	case 0xed: return PREFIX_ED;
	case 0xcb: return PREFIX_CB;
	case 0xdd: return PREFIX_DD;
	case 0xfd: return PREFIX_FD;
	default:
		break;
	}
	return 0;
}

void cpu_step(void)
{
	unsigned int pbit, prefix = 0;
	uint8_t op;

	if(halt) return;

	if(dbg_begin_instr(regs.pc) == -1) {
		emu_break();
		return;
	}

	prev_instr_ei = 0;

	op = fetch_byte();
	regs.r = (regs.r & 0x80) | ((regs.r + 1) & 0x7f);

	if((pbit = prefix_bit(op))) {
		prefix = pbit;
		op = fetch_byte();

		/* only treat the next byte as another prefix if the previous was dd or fd */
		if((pbit = prefix_bit(op)) && (prefix == 0xdd || prefix == 0xfd)) {
			prefix |= pbit;
			op = fetch_byte();
		}
	}

	if(runop[prefix]) {
		runop[prefix](op);
	}

	dbg_end_instr();
}

void cpu_intr(void)
{
	uint8_t data;
	uint16_t addr;

	if(!(regs.iff & 1)) return;

	halt = 0;
	regs.iff = 0;

	switch(regs.imode) {
	case 0:
		data = emu_intr_ack();
		runop_main(data);	/* TODO add support for multi-byte instructions */
		break;

	case 1:
		op_call(0x38);
		break;

	case 2:
		data = emu_intr_ack();
		addr = (uint16_t)data | ((uint16_t)regs.i << 8);
		addr = (uint16_t)emu_mem_read(addr) | ((uint16_t)emu_mem_read(addr + 1) << 8);
		op_call(addr);
		break;
	}
}

struct registers *cpu_regs(void)
{
	return &regs;
}

int cpu_set_named(const char *name, unsigned int val)
{
	int i;

	for(i=0; r8str[i]; i++) {
		if(strcmp(name, r8str[i]) == 0) {
			set_reg8(i, val);
			return 0;
		}
	}

	for(i=0; r16str[i]; i++) {
		if(strcmp(name, r16str[i]) == 0) {
			set_reg16(i, val);
			return 0;
		}
	}

	return -1;
}

int cpu_get_named(const char *name)
{
	int i;

	for(i=0; r8str[i]; i++) {
		if(strcmp(name, r8str[i]) == 0) {
			return get_reg8(i);
		}
	}

	for(i=0; r16str[i]; i++) {
		if(strcmp(name, r16str[i]) == 0) {
			return get_reg16(i);
		}
	}

	return -1;
}

int cpu_get_halt(void)
{
	return halt;
}

int cpu_get_intr(void)
{
	return (regs.iff & 1) && !prev_instr_ei;
}

static int cond(int cc)
{
	switch(cc) {
	case CC_NZ: return ~regs.g.r.f & FLAGS_Z;
	case CC_Z:  return regs.g.r.f & FLAGS_Z;
	case CC_NC: return ~regs.g.r.f & FLAGS_C;
	case CC_C:  return regs.g.r.f & FLAGS_C;
	case CC_PO: return ~regs.g.r.f & FLAGS_PV;
	case CC_PE: return regs.g.r.f & FLAGS_PV;
	case CC_P:  return ~regs.g.r.f & FLAGS_S;
	case CC_M:  return regs.g.r.f & FLAGS_S;
	default:
		break;
	}
	return 0;
}

#define ALUOP(x)		(((x) >> 3) & 7)
#define DEST_R(x)		(((x) >> 3) & 7)
#define OPCOND(x)		(((x) >> 3) & 7)
#define SRC_R(x)		((x) & 7)
#define OP_RR(x)		(((x) >> 4) & 3)
#define RST_ADDR(x)		((x) & 0x38)

#define SWAP_RR(a, b) \
	do { \
		uint16_t tmp = a; \
		a = b; \
		b = tmp; \
	} while(0)

static void runop_main(uint8_t op)
{
	int b67 = op >> 6;
	int8_t disp;
	uint8_t val8, prev_f;
	uint16_t val16, addr;

	switch(op) {
		/* 8-bit load group except ld r,r/ld r,imm/ld r,(hl)/ld (hl),r in default */
	case 0x36:
		val8 = fetch_byte();
		dbg_log_instr("ld (hl), $%x", (unsigned int)val8);
		op_store_mem_imm8(regs.g.rr.hl, val8);
		break;
	case 0x0a:
		dbg_log_instr("ld a, (bc)");
		op_load_reg8_mem(R_A, regs.g.rr.bc);
		break;
	case 0x1a:
		dbg_log_instr("ld a, (de)");
		op_load_reg8_mem(R_A, regs.g.rr.de);
		break;
	case 0x3a:
		val16 = fetch_imm16();
		dbg_log_instr("ld a, $%x", (unsigned int)val16);
		op_load_reg8_mem(R_A, val16);
		break;
	case 0x02:
		dbg_log_instr("ld (bc), a");
		op_store_mem_reg8(regs.g.rr.bc, R_A);
		break;
	case 0x12:
		dbg_log_instr("ld (de), a");
		op_store_mem_reg8(regs.g.rr.de, R_A);
		break;
	case 0x32:
		val16 = fetch_imm16();
		dbg_log_instr("ld ($%x), a", (unsigned int)val16);
		op_store_mem_reg8(val16, R_A);
		break;

		/* 16-bit load group */
	case 0x01:
	case 0x11:
	case 0x21:
	case 0x31:
		val16 = fetch_imm16();
		dbg_log_instr("ld %s, $%x", r16str[OP_RR(op)], (unsigned int)val16);
		op_load_reg16_imm16(OP_RR(op), val16);
		break;
	case 0x2a:
		addr = fetch_imm16();
		dbg_log_instr("ld hl, ($%x)", (unsigned int)addr);
		op_load_reg16_mem(RR_HL, addr);
		break;
	case 0x22:
		addr = fetch_imm16();
		dbg_log_instr("ld ($%x), hl", (unsigned int)addr);
		op_store_mem_reg16(addr, RR_HL);
		break;
	case 0xf9:
		dbg_log_instr("ld sp, hl");
		op_load_reg16_reg16(RR_SP, RR_HL);
		break;
	case 0xc5:
	case 0xd5:
	case 0xe0:
	case 0xf5:
		dbg_log_instr("push %s", r16str[OP_RR(op)]);
		op_push_reg16(RRSET2 | OP_RR(op));
		break;
	case 0xc1:
	case 0xd1:
	case 0xe1:
	case 0xf1:
		dbg_log_instr("pop %s", r16str[OP_RR(op)]);
		op_pop_reg16(RRSET2 | OP_RR(op));
		break;

		/* exchange, block transfer, block search groups */
	case 0xeb:
		dbg_log_instr("ex de, hl");
		SWAP_RR(regs.g.rr.de, regs.g.rr.hl);
		break;
	case 0x08:
		dbg_log_instr("ex af, af'");
		SWAP_RR(regs.g.rr.af, regs.shadow.rr.af);
		break;
	case 0xd9:
		dbg_log_instr("exx");
		SWAP_RR(regs.g.rr.bc, regs.shadow.rr.bc);
		SWAP_RR(regs.g.rr.de, regs.shadow.rr.de);
		SWAP_RR(regs.g.rr.hl, regs.shadow.rr.hl);
		break;
	case 0xe3:
		dbg_log_instr("ex (sp), hl");
		op_exch_mem_reg16(regs.sp, RR_HL);
		break;

		/* general purpose arithmetic and cpu control groups */
	case 0x27:
		/*dbg_log_instr("daa");*/
		break;	/* TODO implement DAA */
	case 0x2f:	/* cpl a */
		dbg_log_instr("cpl a");
		regs.g.r.a = ~regs.g.r.a;
		regs.g.r.f |= FLAGS_H | FLAGS_N;
		break;
	case 0x3f:	/* ccf */
		dbg_log_instr("ccf");
		regs.g.r.f ^= FLAGS_C;
		break;
	case 0x37:	/* scf */
		dbg_log_instr("scf");
		regs.g.r.f |= FLAGS_C;
	case 0x00:	/* nop */
		dbg_log_instr("nop");
		break;
	case 0x76:	/* halt */
		dbg_log_instr("halt");
		halt = 1;
		break;
	case 0xf3:	/* di */
		dbg_log_instr("di");
		regs.iff = 0;
		break;
	case 0xfb:	/* ei */
		dbg_log_instr("ei");
		regs.iff = 3;
		prev_instr_ei = 1;
		break;

		/* 16-bit arithmetic group */
	case 0x09:
	case 0x19:
	case 0x29:
	case 0x39:
		dbg_log_instr("add hl, %s", r16str[OP_RR(op)]);
		op_add_reg16_reg16(RR_HL, OP_RR(op));
		break;
	case 0x03:
	case 0x13:
	case 0x23:
	case 0x33:
		dbg_log_instr("inc %s", r16str[OP_RR(op)]);
		op_incdec_reg16(OP_RR(op), 1);
		break;
	case 0x0b:
	case 0x1b:
	case 0x2b:
	case 0x3b:
		dbg_log_instr("dec %s", r16str[OP_RR(op)]);
		op_incdec_reg16(OP_RR(op), -1);
		break;

		/* rotate and shift group */
	case 0x07:	/* rlca */
		dbg_log_instr("rlca");
		set_reg8(R_A, sr_left(get_reg8(R_A), SR_ROT_NOCARRY));
		break;
	case 0x17:	/* rla */
		dbg_log_instr("rla");
		set_reg8(R_A, sr_left(get_reg8(R_A), SR_ROT_CARRY));
		break;
	case 0x0f:	/* rrca */
		dbg_log_instr("rrca");
		set_reg8(R_A, sr_right(get_reg8(R_A), SR_ROT_NOCARRY));
		break;
	case 0x1f:	/* rra */
		dbg_log_instr("rra");
		set_reg8(R_A, sr_right(get_reg8(R_A), SR_ROT_CARRY));
		break;

		/* jump group except jp cc, imm16 in default */
	case 0xc3:	/* jp imm16 */
		regs.pc = fetch_imm16();
		dbg_log_instr("jp $%x", (unsigned int)regs.pc);
		break;
	case 0x18:	/* jr imm8 */
		disp = fetch_sbyte();
		regs.pc += disp;
		dbg_log_instr("jr %d", (int)disp);
		break;
	case 0x38:	/* jr c, imm8 */
		disp = fetch_sbyte();
		if(cond(CC_C)) regs.pc += disp;
		dbg_log_instr("jr c, %d", (int)disp);
		break;
	case 0x30:	/* jr nc, imm8 */
		disp = fetch_sbyte();
		if(cond(CC_NC)) regs.pc += disp;
		dbg_log_instr("jr nc, %d", (int)disp);
		break;
	case 0x28:	/* jr z, imm8 */
		disp = fetch_sbyte();
		if(cond(CC_Z)) regs.pc += disp;
		dbg_log_instr("jr z, %d", (int)disp);
		break;
	case 0x20:	/* jr nz, imm8 */
		disp = fetch_sbyte();
		if(cond(CC_NZ)) regs.pc += disp;
		dbg_log_instr("jr nz, %d", (int)disp);
		break;
	case 0xe9:	/* jp (hl) */
		regs.pc = regs.g.rr.hl;
		dbg_log_instr("jp (hl)");
		break;
	case 0x10:	/* djnz, imm8 */
		disp = fetch_sbyte();
		if(--regs.g.r.b) regs.pc += disp;
		dbg_log_instr("djnz, %d", (int)disp);
		break;

		/* call and return group except call cc,imm16/ret cc,imm16/rst */
	case 0xcd:
		addr = fetch_imm16();
		dbg_log_instr("call $%x", (unsigned int)addr);
		op_call(addr);
		break;
	case 0xc9:
		dbg_log_instr("ret");
		op_ret();
		break;

		/* input and output group */
	case 0xdb:
		val8 = fetch_byte();
		dbg_log_instr("in a, ($%x)", (unsigned int)val8);
		prev_f = regs.g.r.f;	/* preserve flags */
		op_input(R_A, ((uint16_t)regs.g.r.a << 8) | val8);
		regs.g.r.f = prev_f;
		break;
	case 0xd3:
		val8 = fetch_byte();
		dbg_log_instr("out ($%x), a", (unsigned int)val8);
		op_output(((uint16_t)regs.g.r.a << 8) | val8, R_A);
		break;

	default:
		switch(b67) {
		case 0:
			if(SRC_R(op) == 6) {
				val8 = fetch_byte();
				dbg_log_instr("ld %s, $%x", r8str[DEST_R(op)], val8);
				op_load_reg8_imm8(DEST_R(op), val8);

			} else if(SRC_R(op) == 4 || SRC_R(op) == 5) {
				int adj = (op & 1) ? -1 : 1;
				if(DEST_R(op) != 6) {
					dbg_log_instr("%s %s", adj > 0 ? "inc" : "dec", r8str[DEST_R(op)]);
					op_incdec_reg8(DEST_R(op), adj);
				} else {
					dbg_log_instr("%s (hl)", adj > 0 ? "inc" : "dec");
					op_incdec_mem(regs.g.rr.hl, adj);
				}
			}
			break;

		case 1:
			if(DEST_R(op) != 6 && SRC_R(op) != 6) {
				dbg_log_instr("ld %s, %s", r8str[DEST_R(op)], r8str[SRC_R(op)]);
				op_load_reg8_reg8(DEST_R(op), SRC_R(op));
			} else {
				if(SRC_R(op) == 6) {
					dbg_log_instr("ld %s, (hl)", r8str[DEST_R(op)]);
					op_load_reg8_mem(DEST_R(op), regs.g.rr.hl);
				} else if(DEST_R(op) == 6) {
					dbg_log_instr("ld (hl), %s", r8str[SRC_R(op)]);
					op_store_mem_reg8(regs.g.rr.hl, SRC_R(op));
				}
			}
			break;

		case 2:
			if(SRC_R(op) == 6) {
				dbg_log_instr("%s (hl)", aluopstr[ALUOP(op)]);
				op_alu_mem(ALUOP(op), regs.g.rr.hl);
			} else {
				dbg_log_instr("%s %s", aluopstr[ALUOP(op)], r8str[SRC_R(op)]);
				op_alu_reg8(ALUOP(op), SRC_R(op));
			}
			break;

		case 3:
			if(SRC_R(op) == 6) {	/* alu-op a, imm8 */
				val8 = fetch_byte();
				dbg_log_instr("%s $%x", aluopstr[ALUOP(op)], (unsigned int)val8);
				op_alu_imm8(ALUOP(op), val8);

			} else if(SRC_R(op) == 2) { /* jp cc, imm16 */
				addr = fetch_imm16();
				dbg_log_instr("jp %s, $%x", ccstr[OPCOND(op)], (unsigned int)addr);
				if(cond(OPCOND(op))) {
					regs.pc = addr;
				}

			} else if(SRC_R(op) == 4) { /* call cc, imm16 */
				addr = fetch_imm16();
				dbg_log_instr("call %s, $%x", ccstr[OPCOND(op)], (unsigned int)addr);
				if(cond(OPCOND(op))) op_call(addr);

			} else if(SRC_R(op) == 0) { /* ret cc */
				dbg_log_instr("ret %s", ccstr[OPCOND(op)]);
				if(cond(OPCOND(op))) op_ret();

			} else if(SRC_R(op) == 7) { /* rst */
				dbg_log_instr("rst $%x", RST_ADDR(op) >> 3);
				op_call(RST_ADDR(op));
			}
			break;

		default:
			break;
		}

		break;	/* treat any unknown opcodes as nops */
	}
}

static void runop_ed(uint8_t op)
{
	int b67 = op >> 6;
	int b012 = op & 7;
	int b345 = (op >> 3) & 7;
	uint16_t addr;
	int8_t s8;
	uint8_t byte;

	switch(op) {
		/* 8-bit load group */
	case 0x57:	/* ld a, i */
		dbg_log_instr("ld a, i");
		regs.g.r.a = regs.i;
		if(0) {
	case 0x5f:	/* ld a, r */
			dbg_log_instr("ld a, r");
			regs.g.r.a = regs.r;
		}
		set_flag(FLAGS_S, regs.g.r.a & 0x80);
		set_flag(FLAGS_Z, regs.g.r.a == 0);
		set_flag(FLAGS_H, 0);
		set_flag(FLAGS_PV, regs.iff & 2);
		set_flag(FLAGS_N, 0);
		break;
	case 0x47:	/* ld i, a */
		dbg_log_instr("ld i, a");
		regs.i = regs.g.r.a;
		break;
	case 0x4f:	/* ld r, a */
		dbg_log_instr("ld, r, a");
		regs.r = regs.g.r.a;
		break;

		/* 16-bit load group */
	case 0x4b:	/* ld r16, (imm16) */
	case 0x5b:
	case 0x6b:
	case 0x7b:
		addr = fetch_imm16();
		dbg_log_instr("ld %s, ($%04x)", r16str[OP_RR(op)], (unsigned int)addr);
		op_load_reg16_mem(OP_RR(op), addr);
		break;
	case 0x43:	/* ld (imm16), r16 */
	case 0x53:
	case 0x63:
	case 0x73:
		addr = fetch_imm16();
		dbg_log_instr("ld ($%04x), %s", (unsigned int)addr, r16str[OP_RR(op)]);
		op_store_mem_reg16(addr, OP_RR(op));
		break;

		/* exchange, block transfer, block search groups */
	case 0xa0:	/* ldi */
		dbg_log_instr("ldi");
		op_load_incdec(1, LDI_ONCE);
		break;
	case 0xb0:	/* ldir */
		dbg_log_instr("ldir");
		op_load_incdec(1, LDI_REP);
		break;
	case 0xa8:	/* ldd */
		dbg_log_instr("ldd");
		op_load_incdec(-1, LDI_ONCE);
		break;
	case 0xb8:	/* lddr */
		dbg_log_instr("lddr");
		op_load_incdec(-1, LDI_REP);
		break;
	case 0xa1:	/* cpi */
		dbg_log_instr("cpi");
		op_cmp_incdec(1, LDI_ONCE);
		break;
	case 0xb1:	/* cpir */
		dbg_log_instr("cpir");
		op_cmp_incdec(1, LDI_REP);
		break;
	case 0xa9:	/* cpd */
		dbg_log_instr("cpd");
		op_cmp_incdec(-1, LDI_ONCE);
		break;
	case 0xb9:	/* cpdr */
		dbg_log_instr("cpdr");
		op_cmp_incdec(-1, LDI_REP);
		break;

		/* general-purpose arithmetic and cpu control groups */
	case 0x44:	/* neg */
	case 0x54:
	case 0x64:
	case 0x74:
	case 0x4c:
	case 0x5c:
	case 0x6c:
	case 0x7c:
		dbg_log_instr("neg");
		s8 = get_reg8s(R_A);
		set_reg8(R_A, -s8);
		set_flag(FLAGS_S, regs.g.r.a & 0x80);
		set_flag(FLAGS_Z, regs.g.r.a == 0);
		set_flag(FLAGS_H, (-s8 & 0xf) & 0x10);
		set_flag(FLAGS_PV, s8 == 0x80);
		set_flag(FLAGS_C, s8 != 0);
		break;
	case 0x46:	/* im 0 */
	case 0x66:
	case 0x4e:
	case 0x6e:
		dbg_log_instr("im 0");
		regs.imode = 0;
		break;
	case 0x56:	/* im 1 */
	case 0x76:
		dbg_log_instr("im 1");
		regs.imode = 1;
		break;
	case 0x5e:	/* im 2 */
	case 0x7e:
		dbg_log_instr("im 2");
		regs.imode = 2;
		break;

		/* 16-bit arithmetic group */
	case 0x4a:	/* adc hl, reg16 */
	case 0x5a:
	case 0x6a:
	case 0x7a:
		dbg_log_instr("adc hl, %s", r16str[OP_RR(op)]);
		op_adc_sbc_reg16_reg16(RR_HL, OP_RR(op), 0);
		break;
	case 0x42:	/* sbc hl, reg16 */
	case 0x52:
	case 0x62:
	case 0x72:
		dbg_log_instr("sbc hl, %s", r16str[OP_RR(op)]);
		op_adc_sbc_reg16_reg16(RR_HL, OP_RR(op), -1);
		break;

		/* shift and rotate group */
	case 0x6f:	/* rld */
		dbg_log_instr("rld");
		byte = emu_mem_read(regs.g.rr.hl);
		emu_mem_write(regs.g.rr.hl, (byte << 4) | (regs.g.r.a & 0xf));
		regs.g.r.a = (regs.g.r.a & 0xf0) | (byte >> 4);
		set_flag(FLAGS_S, regs.g.r.a & 0x80);
		set_flag(FLAGS_Z, regs.g.r.a == 0);
		set_flag(FLAGS_H, 0);
		set_flag(FLAGS_PV, parity(regs.g.r.a));
		set_flag(FLAGS_N, 0);
		break;
	case 0x67:	/* rrd */
		dbg_log_instr("rrd");
		byte = emu_mem_read(regs.g.rr.hl);
		emu_mem_write(regs.g.rr.hl, (byte >> 4) | (regs.g.r.a << 4));
		regs.g.r.a = (regs.g.r.a & 0xf0) | (byte & 0xf);
		set_flag(FLAGS_S, regs.g.r.a & 0x80);
		set_flag(FLAGS_Z, regs.g.r.a  == 0);
		set_flag(FLAGS_H, 0);
		set_flag(FLAGS_PV, parity(regs.g.r.a));
		set_flag(FLAGS_N, 0);
		break;

		/* call and return group */
	case 0x4d:	/* reti */
		dbg_log_instr("reti");
		op_ret();
		break;
	case 0x45:	/* retn */
		dbg_log_instr("retn");
		op_ret();
		regs.iff = (regs.iff & 0xfe) | ((regs.iff & 2) >> 1);
		break;

		/* input and output group */
	case 0xa2:	/* ini */
		dbg_log_instr("ini");
		op_input_incdec(1, LDI_ONCE);
		break;
	case 0xb2:	/* inir */
		dbg_log_instr("inir");
		op_input_incdec(1, LDI_REP);
		break;
	case 0xaa:	/* ind */
		dbg_log_instr("ind");
		op_input_incdec(-1, LDI_ONCE);
		break;
	case 0xba:	/* indr */
		dbg_log_instr("indr");
		op_input_incdec(-1, LDI_REP);
		break;
	case 0xa3:	/* outi */
		dbg_log_instr("outi");
		op_output_incdec(1, LDI_ONCE);
		break;
	case 0xb3:	/* otir */
		dbg_log_instr("otir");
		op_output_incdec(1, LDI_REP);
		break;
	case 0xab:	/* outd */
		dbg_log_instr("outd");
		op_output_incdec(-1, LDI_ONCE);
		break;
	case 0xbb:	/* otdr */
		dbg_log_instr("otdr");
		op_output_incdec(-1, LDI_REP);
		break;

	default:
		if(b67 == 1) {
			if(b012 == 0) {
				/* in r, (c) */
				dbg_log_instr("in %s, (c)", b345 == 6 ? "?" : r8str[b345]);
				op_input(b345, regs.g.rr.bc);
			} else if(b012 == 1) {
				/* out (c), r */
				dbg_log_instr("out (c), %s", b345 == 6 ? "0" : r8str[b345]);
				op_output(regs.g.rr.bc, b345);
			}
		}
		break;
	}
}

static void runop_cb(uint8_t op)
{
	int b67 = op >> 6;
	int b012 = op & 7;
	int b345 = (op >> 3) & 7;
	uint16_t addr;

	switch(op) {
		/* rotate and shift group */
	case 0x06:	/* rlc (hl) */
		addr = regs.g.rr.hl;
		dbg_log_instr("rlc (hl)");
		emu_mem_write(addr, sr_left(emu_mem_read(addr), SR_ROT_NOCARRY | SR_SETFL));
		break;
	case 0x0e:	/* rrc (hl) */
		addr = regs.g.rr.hl;
		dbg_log_instr("rrc (hl)");
		emu_mem_write(addr, sr_right(emu_mem_read(addr), SR_ROT_NOCARRY | SR_SETFL));
		break;
	case 0x16:	/* rl (hl) */
		addr = regs.g.rr.hl;
		dbg_log_instr("rl (hl)");
		emu_mem_write(addr, sr_left(emu_mem_read(addr), SR_ROT_CARRY | SR_SETFL));
		break;
	case 0x1e:	/* rr (hl) */
		addr = regs.g.rr.hl;
		dbg_log_instr("rr (hl)");
		emu_mem_write(addr, sr_right(emu_mem_read(addr), SR_ROT_CARRY | SR_SETFL));
		break;
	case 0x26:	/* sla (hl) */
	case 0x36:	/* sll (hl) - undocumented */
		addr = regs.g.rr.hl;
		dbg_log_instr("sla (hl)");
		emu_mem_write(addr, sr_left(emu_mem_read(addr), SR_SETFL));
		break;
	case 0x2e:	/* sra (hl) */
		addr = regs.g.rr.hl;
		dbg_log_instr("sra (hl)");
		emu_mem_write(addr, sr_right(emu_mem_read(addr), SR_SIGN_EXT | SR_SETFL));
		break;
	case 0x3e:	/* srl (hl) */
		addr = regs.g.rr.hl;
		dbg_log_instr("srl (hl)");
		emu_mem_write(addr, sr_right(emu_mem_read(addr), SR_SETFL));
		break;

	default:
		if(b67 == 0) {
			/* rotate/shift */
			switch(b345) {
			case 0:	/* rlc r */
				dbg_log_instr("rlc %s", r8str[b012]);
				set_reg8(b012, sr_left(get_reg8(b012), SR_ROT_NOCARRY | SR_SETFL));
				break;
			case 1:	/* rrc r */
				dbg_log_instr("rrc %s", r8str[b012]);
				set_reg8(b012, sr_right(get_reg8(b012), SR_ROT_NOCARRY | SR_SETFL));
				break;
			case 2:	/* rl r */
				dbg_log_instr("rl %s", r8str[b012]);
				set_reg8(b012, sr_left(get_reg8(b012), SR_ROT_CARRY | SR_SETFL));
				break;
			case 3: /* rr r */
				dbg_log_instr("rr %s", r8str[b012]);
				set_reg8(b012, sr_right(get_reg8(b012), SR_ROT_CARRY | SR_SETFL));
				break;
			case 4:	/* sla r */
				dbg_log_instr("sla %s", r8str[b012]);
				set_reg8(b012, sr_left(get_reg8(b012), SR_SETFL));
				break;
			case 5:	/* sra r */
				dbg_log_instr("sra %s", r8str[b012]);
				set_reg8(b012, sr_right(get_reg8(b012), SR_SETFL | SR_SIGN_EXT));
				break;
			case 7:	/* srl r */
				dbg_log_instr("srl %s", r8str[b012]);
				set_reg8(b012, sr_right(get_reg8(b012), SR_SETFL));
				break;
			default:
				break;
			}
		} else if(b67 == 1) {
			/* bit test */
			uint8_t val;
			if(b012 == 6) {
				val = emu_mem_read(regs.g.rr.hl);
				dbg_log_instr("bit %d, (hl)", (int)b345);
			} else {
				val = get_reg8(b012);
				dbg_log_instr("bit %d, %s", (int)b345, r8str[b012]);
			}
			set_flag(FLAGS_Z, (val & (1 << b345)) == 0);
			set_flag(FLAGS_H, 1);
			set_flag(FLAGS_N, 0);

		} else {
			/* bit set (b67 == 3) / bit reset (b67 == 2) */
			uint8_t val = b012 == 6 ? emu_mem_read(regs.g.rr.hl) : get_reg8(b012);
			if(b67 == 3) {
				val |= 1 << b345;
			} else {
				val &= ~(1 << b345);
			}
			if(b012 == 6) {
				emu_mem_write(regs.g.rr.hl, val);
				dbg_log_instr("%s %d, (hl)", b67 == 3 ? "set" : "res", (int)b345);
			} else {
				set_reg8(b012, val);
				dbg_log_instr("%s %d, %s", b67 == 3 ? "set" : "res", (int)b345, r8str[b012]);
			}
		}
	}
}


static void runop_dd_or_fd(uint8_t op, uint8_t prefix)
{
	uint16_t addr;
	int8_t disp;
	uint8_t byte;
	uint16_t imm16;
	int b345 = (op >> 3) & 7;
	int b012 = op & 7;
	const char *iregstr;
	uint16_t *regptr;
	unsigned int rrx_idx, rrset_bit;

	if(prefix == 0xdd) {
		iregstr = "ix";
		regptr = &regs.ix;
		rrset_bit = RRSET_IX;
	} else {
		iregstr = "iy";
		regptr = &regs.iy;
		rrset_bit = RRSET_IY;
	}
	rrx_idx = rrset_bit | RRX_IX;

	switch(op) {
	case 0x36:	/* ld (ix+d), imm8 */
		disp = fetch_sbyte();
		byte = fetch_byte();
		dbg_log_instr("ld (%s+%d), $%02x", iregstr, (int)disp, byte);
		op_store_mem_imm8(*regptr + disp, byte);
		break;
	case 0x21:	/* ld ix, imm16 */
		imm16 = fetch_imm16();
		dbg_log_instr("ld %s, $%04x", iregstr, (unsigned int)imm16);
		*regptr = imm16;
		break;
	case 0x2a:	/* ld ix, (imm16) */
		addr = fetch_imm16();
		dbg_log_instr("ld %s, ($%04x)", iregstr, (unsigned int)addr);
		*regptr = emu_mem_read(addr) | ((uint16_t)emu_mem_read(addr + 1) << 8);
		break;
	case 0x22:	/* ld (imm16), ix */
		addr = fetch_imm16();
		dbg_log_instr("ld ($%04x), %s", (unsigned int)addr, iregstr);
		emu_mem_write(addr, *regptr);
		emu_mem_write(addr + 1, *regptr >> 8);
		break;
	case 0xf9:	/* ld sp, ix */
		dbg_log_instr("ld sp, %s", iregstr);
		regs.sp = *regptr;
		break;
	case 0xe5:	/* push ix */
		dbg_log_instr("push %s", iregstr);
		push(*regptr);
		break;
	case 0xe1:	/* pop ix */
		dbg_log_instr("pop %s", iregstr);
		*regptr = pop();
		break;

		/* exchange, block transfer, block search groups */
	case 0xe3:	/* ex (sp), ix */
		dbg_log_instr("ex (sp), %s", iregstr);
		imm16 = emu_mem_read(regs.sp) | ((uint16_t)emu_mem_read(regs.sp + 1) << 8);
		emu_mem_write(regs.sp, *regptr);
		emu_mem_write(regs.sp, *regptr >> 8);
		*regptr = imm16;
		break;

		/* 8-bit arithmetic & logic group */
	case 0x34:	/* inc (ix+d) */
		disp = fetch_sbyte();
		dbg_log_instr("inc (%s+%d)", iregstr, (int)disp);
		op_incdec_mem(*regptr + disp, 1);
		break;
	case 0x35:	/* dec (ix+d) */
		disp = fetch_sbyte();
		dbg_log_instr("dec (%s+%d)", iregstr, (int)disp);
		op_incdec_mem(*regptr + disp, -1);
		break;

		/* 16-bit arithmetic group */
	case 0x49:	/* add ix, rr */
	case 0x59:
	case 0x69:
	case 0x79:
		dbg_log_instr("add %s, %s", iregstr, r16str[rrset_bit | OP_RR(op)]);
		op_add_reg16_reg16(rrx_idx, rrset_bit | OP_RR(op));
		break;
	case 0x23:	/* inc ix */
		dbg_log_instr("inc %s", iregstr);
		op_incdec_reg16(rrx_idx, 1);
		break;
	case 0x2b:	/* dec ix */
		dbg_log_instr("dec %s", iregstr);
		op_incdec_reg16(rrx_idx, -1);
		break;

		/* jump group */
	case 0xe9:	/* jp (ix) */
		dbg_log_instr("jp (%s)", iregstr);
		regs.pc = *regptr;
		break;

	default:
		if((op & 0xc7) == 0x46) {	/* ld r, (ix+d) */
			disp = fetch_sbyte();
			dbg_log_instr("ld %s, (%s+%d)", r8str[b345], iregstr, (int)disp);
			op_load_reg8_mem(b345, *regptr + disp);
		} else if((op & 0xf8) == 0x70) { /* ld (ix+d), r */
			disp = fetch_sbyte();
			dbg_log_instr("ld (%s+%d), %s", iregstr, (int)disp, r8str[b012]);
			op_store_mem_reg8(*regptr + disp, b012);
		} else if((op & 0xc7) == 0x86) {	/* ALUOP a, (ix+d) */
			disp = fetch_sbyte();
			dbg_log_instr("%s a, (%s+%d)", aluopstr[b345], iregstr, (int)disp);
			op_alu_mem(b345, *regptr + disp);
		} else if((op & 0xc7) == 0x06) {	/* (ix) shift/rotate */
			disp = fetch_sbyte();
			addr = *regptr + disp;
			switch(b345) {
			case 0:	/* rlc (ix+d) */
				dbg_log_instr("rlc (%s+%d)", iregstr, (int)disp);
				emu_mem_write(addr, sr_left(emu_mem_read(addr), SR_ROT_NOCARRY | SR_SETFL));
				break;
			case 1:	/* rrc (ix+d) */
				dbg_log_instr("rrc (%s+%d)", iregstr, (int)disp);
				emu_mem_write(addr, sr_right(emu_mem_read(addr), SR_ROT_NOCARRY | SR_SETFL));
				break;
			case 2:	/* rl (ix+d) */
				dbg_log_instr("rl (%s+%d)", iregstr, (int)disp);
				emu_mem_write(addr, sr_left(emu_mem_read(addr), SR_ROT_CARRY | SR_SETFL));
				break;
			case 3:	/* rr (ix+d) */
				dbg_log_instr("rr (%s+%d)", iregstr, (int)disp);
				emu_mem_write(addr, sr_right(emu_mem_read(addr), SR_ROT_CARRY | SR_SETFL));
				break;
			case 4:	/* sla (ix+d) */
			case 5:	/* sll (ix+d) - undocumented */
				dbg_log_instr("sla (%s+%d)", iregstr, (int)disp);
				emu_mem_write(addr, sr_left(emu_mem_read(addr), SR_SETFL));
				break;
			case 6:	/* sra (ix+d) */
				dbg_log_instr("sra (%s+%d)", iregstr, (int)disp);
				emu_mem_write(addr, sr_right(emu_mem_read(addr), SR_SIGN_EXT | SR_SETFL));
				break;
			case 7:	/* srl (ix+d) */
				dbg_log_instr("srl (%s+%d)", iregstr, (int)disp);
				emu_mem_write(addr, sr_right(emu_mem_read(addr), SR_SETFL));
				break;
			}
		}
	}
}

static void runop_dd(uint8_t op)
{
	runop_dd_or_fd(op, 0xdd);
}

static void runop_fd(uint8_t op)
{
	runop_dd_or_fd(op, 0xfd);
}


static void runop_ddcb(uint8_t op)
{
	/* TODO */
}

static void runop_fdcb(uint8_t op)
{
	/* TODO */
}

static void set_reg8(int r, uint8_t val)
{
	*regptr8[r] = val;
}

static void set_reg8s(int r, int8_t val)
{
	*regptr8[r] = *(uint8_t*)&val;
}

static uint8_t get_reg8(int r)
{
	return *regptr8[r];
}

static int8_t get_reg8s(int r)
{
	return *(int8_t*)regptr8[r];
}

static void set_reg16(int r, uint16_t val)
{
	*regptr16[r] = val;
}

static void set_reg16s(int r, int16_t val)
{
	*regptr16[r] = *(uint16_t*)&val;
}

static uint16_t get_reg16(int r)
{
	return *regptr16[r];
}

static int16_t get_reg16s(int r)
{
	return *(int16_t*)regptr16[r];
}

static void set_flag(unsigned int flag, int val)
{
	if(val) {
		regs.g.r.f |= flag;
	} else {
		regs.g.r.f &= ~flag;
	}
}

static int parity(int x)
{
	int i, s = 0;
	for(i=0; i<8; i++) {
		s += x & 1;
		x >>= 1;
	}
	return s & 1;
}

static int overflow(int x)
{
	return x > 127 || x < -128;
}

static int overflow16(int x)
{
	return x > 32767 || x < -32768;
}

static void op_load_reg8_reg8(int rdest, int rsrc)
{
	set_reg8(rdest, get_reg8(rsrc));
}

static void op_load_reg8_imm8(int rdest, uint8_t imm)
{
	set_reg8(rdest, imm);
}

static void op_load_reg8_mem(int rdest, uint16_t addr)
{
	set_reg8(rdest, emu_mem_read(addr));
}

static void op_store_mem_reg8(uint16_t addr, int rsrc)
{
	emu_mem_write(addr, get_reg8(rsrc));
}

static void op_store_mem_imm8(uint16_t addr, uint8_t imm)
{
	emu_mem_write(addr, imm);
}

static void op_load_reg16_imm16(int rdest, uint16_t imm)
{
	set_reg16(rdest, imm);
}

static void op_load_reg16_reg16(int rdest, int rsrc)
{
	set_reg16(rdest, get_reg16(rsrc));
}

static void op_load_reg16_mem(int rdest, uint16_t addr)
{
	uint16_t lsb = emu_mem_read(addr);
	uint16_t msb = emu_mem_read(addr + 1);
	set_reg16(rdest, lsb | (msb << 8));
}

static void op_store_mem_reg16(uint16_t addr, int rsrc)
{
	uint16_t val = get_reg16(rsrc);
	emu_mem_write(addr, val);
	emu_mem_write(addr + 1, val >> 8);
}

static void op_exch_mem_reg16(uint16_t addr, int rr)
{
	uint16_t val = get_reg16(rr);
	uint16_t lsb = emu_mem_read(addr);
	uint16_t msb = emu_mem_read(addr + 1);
	set_reg16(rr, lsb | (msb << 8));
	emu_mem_write(addr, val);
	emu_mem_write(addr + 1, val >> 8);
}

#define CARRY (regs.g.r.f & 1)

static void op_alu_reg8(int op, int r)
{
	op_alu_imm8(op, get_reg8(r));
}

static void op_alu_mem(int op, uint16_t addr)
{
	op_alu_imm8(op, emu_mem_read(addr));
}

static void op_alu_imm8(int op, uint8_t imm)
{
	int c, h, pv, n = 0;
	int acc = get_reg8s(R_A);
	int rval = *(int8_t*)&imm;
	c = CARRY;

	switch(op) {
	case ALUOP_ADD:
		if(0) {
	case ALUOP_SUB:
	case ALUOP_CP:
			rval = -rval;
			c = -c;
			n = 1;
		}
		h = ((acc & 0xf) + (rval + 0xf)) & 0x10;
		acc += rval;
		c = acc & 0x100;
		pv = overflow(acc);
		break;

	case ALUOP_ADC:
		if(0) {
	case ALUOP_SBC:
			rval = -rval;
			c = -c;
			n = 1;
		}
		h = ((acc & 0xf) + (rval + 0xf) + CARRY) & 0x10;
		acc += rval + CARRY;
		c = acc & 0x100;
		pv = overflow(acc);
		break;

	case ALUOP_AND:
		acc &= rval;
		c = 0;
		h = 1;
		pv = parity(acc);
		break;
	case ALUOP_XOR:
		acc ^= rval;
		c = 0;
		h = 0;
		pv = parity(acc);
		break;
	case ALUOP_OR:
		acc |= rval;
		c = 0;
		h = 0;
		pv = parity(acc);
		break;
	default:
		return;
	}

	set_flag(FLAGS_S, acc & 0x80);
	set_flag(FLAGS_Z, acc == 0);
	set_flag(FLAGS_H, h);
	set_flag(FLAGS_PV, pv);
	set_flag(FLAGS_N, n);
	set_flag(FLAGS_C, c);

	if(op != ALUOP_CP) {
		set_reg8s(R_A, acc);
	}
}

static void op_incdec_reg8(int r, int adj)
{
	int prev = get_reg8s(r);
	int val = prev + adj;

	set_flag(FLAGS_S, val & 0x80);
	set_flag(FLAGS_Z, val == 0);
	set_flag(FLAGS_H, ((prev & 0xf) + adj) & 0x10);
	set_flag(FLAGS_PV, overflow(val));
	set_flag(FLAGS_N, adj < 0);

	set_reg8s(r, val);
}

static void op_incdec_reg16(int r, int adj)
{
	set_reg16s(r, get_reg16s(r) + adj);
}

static void op_incdec_mem(uint16_t addr, int adj)
{
	uint16_t lsb = emu_mem_read(addr);
	uint16_t msb = emu_mem_read(addr + 1);
	int prev, val;

	lsb |= msb << 8;
	prev = *(int16_t*)&lsb;
	val = prev + adj;

	set_flag(FLAGS_S, val & 0x8000);
	set_flag(FLAGS_Z, val == 0);
	set_flag(FLAGS_H, ((prev & 0xfff) + adj) & 0x1000);
	set_flag(FLAGS_PV, overflow16(val));
	set_flag(FLAGS_N, adj < 0);

	lsb = *(uint16_t*)&val;
	emu_mem_write(addr, lsb);
	emu_mem_write(addr + 1, lsb >> 8);
}

static void op_add_reg16_reg16(int rdest, int rsrc)
{
	int16_t a = get_reg16s(rdest);
	int16_t b = get_reg16s(rsrc);

	set_reg16s(rdest, a + b);
	set_flag(FLAGS_N, 0);
}

static void op_adc_sbc_reg16_reg16(int rdest, int rsrc, int sub)
{
	int a = get_reg16s(rdest);
	int b = get_reg16s(rsrc) + CARRY;
	int res;

	if(sub) b = -b;
	res = a + b;
	set_reg16s(rdest, res);

	set_flag(FLAGS_S, res < 0);
	set_flag(FLAGS_Z, res == 0);
	set_flag(FLAGS_H, ((a & 0xfff) + (b & 0xfff)) & 0x1000);
	set_flag(FLAGS_PV, overflow16(res));
	set_flag(FLAGS_N, 0);
	set_flag(FLAGS_C, res & 0x10000);
}

static void push(uint16_t val)
{
	emu_mem_write(--regs.sp, val >> 8);
	emu_mem_write(--regs.sp, val);
}

static uint16_t pop(void)
{
	uint16_t lsb = emu_mem_read(regs.sp++);
	uint16_t msb = emu_mem_read(regs.sp++);
	return lsb | (msb << 8);
}

static void op_push_reg16(int r)
{
	push(get_reg16(r));
}

static void op_pop_reg16(int r)
{
	set_reg16(r, pop());
}

static void op_call(uint16_t addr)
{
	push(regs.pc);
	regs.pc = addr;
}

static void op_ret(void)
{
	regs.pc = pop();
}

static void op_input(int r, uint16_t addr)
{
	uint8_t val = emu_io_read(addr);
	if(r != 6) set_reg8(r, val);

	set_flag(FLAGS_S, val & 0x80);
	set_flag(FLAGS_Z, val == 0);
	set_flag(FLAGS_H, 0);
	set_flag(FLAGS_PV, parity(val));
	set_flag(FLAGS_N, 0);
}

static void op_output(uint16_t addr, int r)
{
	emu_io_write(addr, get_reg8(r));
}

static uint8_t sr_left(uint8_t x, unsigned int mode)
{
	uint8_t b0 = 0;

	if(mode & SR_ROT_CARRY) b0 = CARRY;
	if(mode & SR_ROT_NOCARRY) b0 = x >> 7;

	set_flag(FLAGS_C, x & 0x80);
	set_flag(FLAGS_H, 0);
	set_flag(FLAGS_N, 0);

	x = (x << 1) | b0;
	if(mode & SR_SETFL) {
		set_flag(FLAGS_S, x & 0x80);
		set_flag(FLAGS_Z, x == 0);
		set_flag(FLAGS_PV, parity(x));
	}
	return x;
}

static uint8_t sr_right(uint8_t x, unsigned int mode)
{
	uint8_t b7 = 0;

	if(mode & SR_ROT_CARRY) b7 = CARRY << 7;
	if(mode & SR_ROT_NOCARRY) b7 = (x & 1) << 7;
	if(mode & SR_SIGN_EXT) b7 = x & 0x80;

	set_flag(FLAGS_C, x & 1);
	set_flag(FLAGS_H, 0);
	set_flag(FLAGS_N, 0);

	x = (x >> 1) | b7;
	if(mode & SR_SETFL) {
		set_flag(FLAGS_S, x & 0x80);
		set_flag(FLAGS_Z, x == 0);
		set_flag(FLAGS_PV, parity(x));
	}
	return x;
}

static void op_load_incdec(int step, unsigned int mode)
{
	emu_mem_write(regs.g.rr.de, emu_mem_read(regs.g.rr.hl));
	regs.g.rr.de += step;
	regs.g.rr.hl += step;

	if(--regs.g.rr.bc && mode & LDI_REP) {
		regs.pc -= 2;	/* repeat instruction */
	}

	set_flag(FLAGS_PV, mode & LDI_REP ? 0 : regs.g.rr.bc);
	set_flag(FLAGS_H, 0);
	set_flag(FLAGS_N, 0);
}

static void op_cmp_incdec(int step, unsigned int mode)
{
	int acc = get_reg8s(R_A);
	uint8_t byte = emu_mem_read(regs.g.rr.hl);
	int val = -*(int8_t*)&byte;

	regs.g.rr.hl += step;

	if(--regs.g.rr.bc && acc + val != 0 && mode & LDI_REP) {
		regs.pc -= 2;	/* repeat instruction */
	}

	set_flag(FLAGS_S, acc + val < 0);
	set_flag(FLAGS_Z, acc + val == 0);
	set_flag(FLAGS_H, ((acc & 0xf) + (val + 0xf)) & 0x10);
	set_flag(FLAGS_PV, regs.g.rr.bc);
	set_flag(FLAGS_N, 1);
}

static void op_input_incdec(int step, unsigned int mode)
{
	uint8_t byte = emu_io_read(regs.g.rr.bc);
	emu_mem_write(regs.g.rr.hl, byte);

	regs.g.rr.hl += step;

	if(--regs.g.r.b && mode & LDI_REP) {
		regs.pc -= 2;	/* repeat instruction */
	}

	set_flag(FLAGS_Z, regs.g.r.b == 0);
	set_flag(FLAGS_N, 1);
}

static void op_output_incdec(int step, unsigned int mode)
{
	uint8_t byte = emu_mem_read(regs.g.rr.hl);
	emu_io_write(regs.g.rr.bc, byte);

	regs.g.rr.hl += step;

	if(--regs.g.r.b && mode & LDI_REP) {
		regs.pc -= 2;	/* repeat instruction */
	}

	set_flag(FLAGS_Z, regs.g.r.b == 0);
	set_flag(FLAGS_N, 1);
}
