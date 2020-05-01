#include "duart.h"
#include "emu.h"

enum {
	REG_MRA			= 0x0,
	REG_SRA_CSRA	= 0x1,
	REG_CRA			= 0x2,
	REG_RBA_TBA		= 0x3,
	REG_IPCR_ACR	= 0x4,
	REG_ISR_IMR		= 0x5,
	REG_CUR_CTUR	= 0x6,
	REG_CLR_CTLR	= 0x7,
	REG_MRB			= 0x8,
	REG_SRB_CSRB	= 0x9,
	REG_CRB			= 0xa,
	REG_RBB_TBB		= 0xb,
	REG_IVR			= 0xc,
	REG_IP_OPCR		= 0xd,
	REG_CSTART_OPSET= 0xe,
	REG_CSTOP_OPCLR	= 0xf
};

enum {
	CMD_RX_ONOFF	= 0x03,
	CMD_TX_ONOFF	= 0x0c,
	CMD_RST_MPTR	= 0x10,
	CMD_RST_RX		= 0x20,
	CMD_RST_TX		= 0x30,
	CMD_RST_ERR		= 0x40,
	CMD_RST_BRKINT	= 0x50,
	CMD_START_BRK	= 0x60,
	CMD_STOP_BRK	= 0x70
};

enum {
	STAT_RXRDY		= 0x01,
	STAT_FFULL		= 0x02,
	STAT_TXRDY		= 0x04,
	STAT_TXEMPTY	= 0x08,
	STAT_ERR_OVR	= 0x10,
	STAT_ERR_PAR	= 0x20,
	STAT_ERR_FRM	= 0x40,
	STAT_ERR_BRK	= 0x80
};

struct port {
	uint8_t mode[2];
	uint8_t clksel;
	int modeptr;
	int tx, rx;
	uint8_t rxfifo[4];
	int rxfifo_in, rxfifo_out;

};

static struct port port[2];
static uint8_t reg_ipcr;
static uint8_t reg_auxctl;
static uint8_t reg_istat;
static uint8_t reg_imask;
static uint16_t reg_count;
static uint8_t reg_ivec;
static uint8_t reg_opcr;
static uint8_t oport;

void duart_reset(void)
{
	int i;

	reg_imask = 0;
	reg_istat = 0;
	reg_opcr = 0;
	reg_ivec = 0xf;
	oport = 0;

	for(i=0; i<2; i++) {
		port[i].mode[0] = port[i].mode[1] = 0;
		port[i].modeptr = 0;
		port[i].tx = port[i].rx = 0;
		port[i].rxfifo_in = port[i].rxfifo_out = 0;
	}
}

void duart_serin(int port, int c)
{
}

static int tx_ready(int pidx)
{
	return port[pidx].tx;
}

static int rx_ready(int pidx)
{
	return port[pidx].rxfifo_in != port[pidx].rxfifo_out;
}

static int rx_full(int pidx)
{
	return ((port[pidx].rxfifo_in + 1) & 3) == port[pidx].rxfifo_out;
}

static void command(int pidx, uint8_t data)
{
	if(data & CMD_RX_ONOFF) {
		port[pidx].rx = (data & CMD_RX_ONOFF) == 1;
	}
	if(data & CMD_TX_ONOFF) {
		port[pidx].tx = (data & CMD_TX_ONOFF) == 4;
	}
	switch(data & 0xf0) {
	case CMD_RST_MPTR:
		port[pidx].modeptr = 0;
		break;

	default:
		break;
	}
}

uint8_t duart_read(int rs)
{
	uint8_t res;
	int pidx = (rs >> 3) & 1;

	switch(rs) {
	case REG_SRA_CSRA:
	case REG_SRB_CSRB:
		res = STAT_TXEMPTY | STAT_TXRDY;
		if(rx_ready(pidx)) {
			res |= STAT_RXRDY;
		}
		if(rx_full(pidx)) {
			res |= STAT_FFULL;
		}
		return res;

	default:
		break;
	}
	return 0;
}

void duart_write(int rs, uint8_t data)
{
	int mptr;
	int pidx = (rs >> 3) & 1;

	switch(rs) {
	case REG_CRA:
	case REG_CRB:
		command(pidx, data);
		break;

	case REG_MRA:
	case REG_MRB:
		mptr = port[pidx].modeptr;
		port[pidx].mode[mptr] = data;
		if(!mptr) port[pidx].modeptr = 1;
		break;

	case REG_RBA_TBA:
	case REG_RBB_TBB:
		if(tx_ready(pidx)) {
			emu_serout(pidx, data);
		}
		break;
	}
}
