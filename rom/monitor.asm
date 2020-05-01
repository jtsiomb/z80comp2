; vi:filetype=z80:

	include "uart.inc"
main:
	ld sp, $0

	; --- initialize channel A ---
	; reset TX/RX
	ld a, UART_CMD_RST_RX
	out (UART_REG_CMDA), a
	ld a, UART_CMD_RST_TX
	out (UART_REG_CMDA), a
	; disable TX/RX
	ld a, UART_CMD_TX_OFF | UART_CMD_RX_OFF
	out (UART_REG_CMDA), a
	; reset MODEA pointer
	ld a, UART_CMD_RST_MPTR
	out (UART_REG_CMDA), a
	; set the MODEA register
	ld a, UART_M1_8BIT | UART_M1_NOPAR
	out (UART_REG_MODEA), a
	ld a, UART_M2_STOP1
	out (UART_REG_MODEA), a
	; select baud generator set 1
	xor a
	out (UART_REG_AUXCTL), a
	; set the baud rate
	ld a, UART_CSEL_TX_9600 | UART_CSEL_RX_9600
	out (UART_REG_CSELA), a
	; enable TX/RX
	ld a, UART_CMD_TX_ON | UART_CMD_RX_ON
	out (UART_REG_CMDA), a

	; --- initialize channel B ---
	; reset TX/RX
	ld a, UART_CMD_RST_RX
	out (UART_REG_CMDB), a
	ld a, UART_CMD_RST_TX
	out (UART_REG_CMDB), a
	; disable TX/RX
	ld a, UART_CMD_TX_OFF | UART_CMD_RX_OFF
	out (UART_REG_CMDB), a
	; reset MODEB pointer
	ld a, UART_CMD_RST_MPTR
	out (UART_REG_CMDB), a
	; write all 0 to MODEB registers to leave OP1 as a general purpose output
	xor a
	out (UART_REG_MODEB), a
	out (UART_REG_MODEB), a
	; set the output port bit 1
	ld a, 2
	out (UART_REG_OSET), a

	ld hl, str_hello
	call uart_putstr
	ld hl, str_foo
	call uart_putstr

hlt:	di
	halt
	jr hlt


uart_putchar:
	ld b, a
.wait_txrdy:
	in a, (UART_REG_STATA)
	bit 2, a	; test status bit 2 (TXRDY)
	jr z, .wait_txrdy
	ld a, b
	out (UART_REG_DATAA), a
	ret

uart_putstr:
	ld a, (hl)
	cp a, 0
	ret z
	call uart_putchar
	inc hl
	jr uart_putstr

str_hello asciiz 'The Z80 says hi!',13,10
str_foo asciiz 'Foobar',13,10
