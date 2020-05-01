#include <stdint.h>
#include "cpu.h"
#include "emu.h"

enum {
	FLAGS_C		= 0x01,
	FLAGS_N		= 0x02,
	FLAGS_PV	= 0x04,
	FLAGS_H		= 0x10,
	FLAGS_Z		= 0x40,
	FLAGS_S		= 0x80
};

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

#define RRSET2	4
enum {
	RR_BC = 0,
	RR_DE = 1,
	RR_HL = 2,
	RR_SP = 3,

	RR2_BC = RRSET2 | RR_BC,
	RR2_DE = RRSET2 | RR_DE,
	RR2_HL = RRSET2 | RR_HL,
	RR2_AF = RRSET2 | RR_SP
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

struct regs8 {
	uint8_t a, f, b, c, d, e, h, l;
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

static void runop_main(uint8_t op);
static void runop_ed(uint8_t op);
static void runop_cb(uint8_t op);
static void runop_dd(uint8_t op);
static void runop_fd(uint8_t op);
static void runop_ddcb(uint8_t op);
static void runop_fdcb(uint8_t op);

static void op_load_reg8_reg8(int rdest, int rsrc);
static void op_load_reg8_imm8(int rdest, uint8_t imm);
static void op_load_reg8_mem(int rdest, uint16_t addr);
static void op_store_mem_reg8(uint16_t addr, int rsrc);
static void op_store_mem_imm8(uint16_t addr, uint8_t imm);
static void op_store_mem_reg16(uint16_t addr, int rsrc);
static void op_load_reg16_imm16(int rdest, uint16_t imm);
static void op_load_reg16_reg16(int rdest, int rsrc);
static void op_exch_mem_reg16(uint16_t addr, int rr);
static void op_alu_reg8(int op, int r);
static void op_alu_imm8(int op, uint8_t imm);
static void op_incdec_reg8(int r, int adj);
static void op_incdec_reg16(int r, int adj);
static void op_incdec_mem(uint16_t addr, int adj);
static void op_add_reg16_reg16(int rdest, int rsrc);
static void op_rl_reg8(int r, int width);
static void op_rr_reg8(int r, int width);
static void op_push_reg16(int r);
static void op_pop_reg16(int r);
static void op_call(uint16_t addr);
static void op_ret(void);
static void op_input(int r, uint16_t addr);
static void op_output(uint16_t addr, int r);

static struct registers regs;
static int halt;

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

static uint8_t *regptr8[8] = {
	&regs.g.r.b, &regs.g.r.c,
	&regs.g.r.d, &regs.g.r.e,
	&regs.g.r.h, &regs.g.r.l,
	0, &regs.g.r.a
};

static uint16_t *regptr16[] = {
	&regs.g.rr.bc,
	&regs.g.rr.de,
	&regs.g.rr.hl,
	&regs.sp,

	&regs.g.rr.bc,
	&regs.g.rr.de,
	&regs.g.rr.hl,
	&regs.g.rr.af
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

	op = fetch_byte();
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
	uint8_t disp;
	uint16_t addr;

	switch(op) {
		/* 8-bit load group except ld r,r/ld r,imm/ld r,(hl)/ld (hl),r in default */
	case 0x36:
		op_store_mem_imm8(regs.g.rr.hl, fetch_byte());
		break;
	case 0x0a:
		op_load_reg8_mem(R_A, regs.g.rr.bc);
		break;
	case 0x1a:
		op_load_reg8_mem(R_A, regs.g.rr.de);
		break;
	case 0x3a:
		op_load_reg8_mem(R_A, fetch_imm16());
		break;
	case 0x02:
		op_store_mem_reg8(regs.g.rr.bc, R_A);
		break;
	case 0x12:
		op_store_mem_reg8(regs.g.rr.de, R_A);
		break;
	case 0x32:
		op_store_mem_reg8(fetch_imm16(), R_A);
		break;

		/* 16-bit load group */
	case 0x01:
	case 0x11:
	case 0x21:
	case 0x31:
		op_load_reg16_imm16(OP_RR(op), fetch_imm16());
		break;
	case 0x2a:
		op_load_reg16_imm16(RR_HL, fetch_imm16());
		break;
	case 0x22:
		op_store_mem_reg16(fetch_imm16(), RR_HL);
		break;
	case 0xf9:
		op_load_reg16_reg16(RR_SP, RR_HL);
		break;
	case 0xc5:
	case 0xd5:
	case 0xe0:
	case 0xf5:
		op_push_reg16(RRSET2 | OP_RR(op));
		break;
	case 0xc1:
	case 0xd1:
	case 0xe1:
	case 0xf1:
		op_pop_reg16(RRSET2 | OP_RR(op));
		break;

		/* exchange, block transfer, block search groups */
	case 0xeb:
		SWAP_RR(regs.g.rr.de, regs.g.rr.hl);
		break;
	case 0x08:
		SWAP_RR(regs.g.rr.af, regs.shadow.rr.af);
		break;
	case 0xd9:
		SWAP_RR(regs.g.rr.bc, regs.shadow.rr.bc);
		SWAP_RR(regs.g.rr.de, regs.shadow.rr.de);
		SWAP_RR(regs.g.rr.hl, regs.shadow.rr.hl);
		break;
	case 0xe3:
		op_exch_mem_reg16(regs.sp, RR_HL);
		break;

		/* general purpose arithmetic and cpu control groups */
	case 0x27:
		break;	/* TODO implement DAA */
	case 0x2f:	/* cpl a */
		regs.g.r.a = ~regs.g.r.a;
		regs.g.r.f |= FLAGS_H | FLAGS_N;
		break;
	case 0x3f:	/* ccf */
		regs.g.r.f ^= FLAGS_C;
		break;
	case 0x37:	/* scf */
		regs.g.r.f |= FLAGS_C;
	case 0x00:	/* nop */
		break;
	case 0x76:	/* halt */
		halt = 1;
		break;
	case 0xf3:	/* di */
		regs.iff = 0;
		break;
	case 0xfb:	/* ei */
		regs.iff = 1;
		break;

		/* 16-bit arithmetic group */
	case 0x09:
	case 0x19:
	case 0x29:
	case 0x39:
		op_add_reg16_reg16(RR_HL, OP_RR(op));
		break;
	case 0x03:
	case 0x13:
	case 0x23:
	case 0x33:
		op_incdec_reg16(OP_RR(op), 1);
		break;
	case 0x0b:
	case 0x1b:
	case 0x2b:
	case 0x3b:
		op_incdec_reg16(OP_RR(op), -1);
		break;

		/* rotate and shift group */
	case 0x07:	/* rlca */
		op_rl_reg8(R_A, 8);
		break;
	case 0x17:	/* rla */
		op_rl_reg8(R_A, 9);
		break;
	case 0x0f:	/* rrca */
		op_rr_reg8(R_A, 8);
		break;
	case 0x1f:	/* rra */
		op_rr_reg8(R_A, 9);
		break;

		/* jump group except jp cc, imm16 in default */
	case 0xc3:	/* jp imm16 */
		regs.pc = fetch_imm16();
		break;
	case 0x18:	/* jr imm8 */
		regs.pc += fetch_byte();
		break;
	case 0x38:	/* jr c, imm8 */
		disp = fetch_byte();
		if(cond(CC_C)) regs.pc += disp;
		break;
	case 0x30:	/* jr nc, imm8 */
		disp = fetch_byte();
		if(cond(CC_NC)) regs.pc += disp;
		break;
	case 0x28:	/* jr z, imm8 */
		disp = fetch_byte();
		if(cond(CC_Z)) regs.pc += disp;
		break;
	case 0x20:	/* jr nz, imm8 */
		disp = fetch_byte();
		if(cond(CC_NZ)) regs.pc += disp;
		break;
	case 0xe9:	/* jp (hl) */
		regs.pc = regs.g.rr.hl;
		break;
	case 0x10:	/* djnz, imm8 */
		disp = fetch_byte();
		if(--regs.g.r.b) regs.pc += disp;
		break;

		/* call and return group except call cc,imm16/ret cc,imm16/rst */
	case 0xcd:
		op_call(fetch_imm16());
		break;
	case 0xc9:
		op_ret();
		break;

		/* input and output group */
	case 0xdb:
		op_input(R_A, ((uint16_t)regs.g.r.a << 8) | fetch_byte());
		break;
	case 0xd3:
		op_output(((uint16_t)regs.g.r.a << 8) | fetch_byte(), R_A);
		break;

	default:
		switch(b67) {
		case 0:
			if(SRC_R(op) == 6) {
				op_load_reg8_imm8(DEST_R(op), fetch_byte());
			} else if(SRC_R(op) == 4 || SRC_R(op) == 5) {
				int adj = (op & 1) ? -1 : 1;
				if(DEST_R(op) != 6) {
					op_incdec_reg8(DEST_R(op), adj);
				} else {
					op_incdec_mem(regs.g.rr.hl, adj);
				}
			}
			break;

		case 1:
			if(DEST_R(op) != 6 && SRC_R(op) != 6) {
				op_load_reg8_reg8(DEST_R(op), SRC_R(op));
			} else {
				if(SRC_R(op) == 6) {
					op_load_reg8_mem(DEST_R(op), regs.g.rr.hl);
				} else if(DEST_R(op) == 6) {
					op_store_mem_reg8(regs.g.rr.hl, SRC_R(op));
				}
			}
			break;

		case 2:
			if(SRC_R(op) != 6) {
				op_alu_reg8(ALUOP(op), SRC_R(op));
			}
			break;

		case 3:
			if(SRC_R(op) == 6) {	/* alu-op a, imm8 */
				op_alu_imm8(ALUOP(op), fetch_byte());

			} else if(SRC_R(op) == 2) { /* jp cc, imm16 */
				addr = fetch_imm16();
				if(cond(OPCOND(op))) {
					regs.pc = addr;
				}

			} else if(SRC_R(op) == 4) { /* call cc, imm16 */
				addr = fetch_imm16();
				if(cond(OPCOND(op))) op_call(addr);

			} else if(SRC_R(op) == 0) { /* ret cc, imm16 */
				addr = fetch_imm16();
				if(cond(OPCOND(op))) op_ret();

			} else if(SRC_R(op) == 7) { /* rst */
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
}

static void runop_cb(uint8_t op)
{
}

static void runop_dd(uint8_t op)
{
}

static void runop_fd(uint8_t op)
{
}

static void runop_ddcb(uint8_t op)
{
}

static void runop_fdcb(uint8_t op)
{
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

static void op_store_mem_reg16(uint16_t addr, int rsrc)
{
	uint16_t val = get_reg16(rsrc);
	emu_mem_write(addr, val);
	emu_mem_write(addr + 1, val >> 8);
}

static void op_load_reg16_imm16(int rdest, uint16_t imm)
{
	set_reg16(rdest, imm);
}

static void op_load_reg16_reg16(int rdest, int rsrc)
{
	set_reg16(rdest, get_reg16(rsrc));
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
}

static void op_rl_reg8(int r, int width)
{
}

static void op_rr_reg8(int r, int width)
{
}

static void op_push_reg16(int r)
{
}

static void op_pop_reg16(int r)
{
}

static void op_call(uint16_t addr)
{
}

static void op_ret(void)
{
}

static void op_input(int r, uint16_t addr)
{
	set_reg8(r, emu_io_read(addr));
}

static void op_output(uint16_t addr, int r)
{
	emu_io_write(addr, get_reg8(r));
}
