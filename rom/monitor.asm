; vi:filetype=z80:
	include "uart.inc"

INPQ_SIZE equ 16
INPQ_HEAD equ $8000
INPQ_TAIL equ $8002
INPQ_BASE equ $8100
IVT_BASE equ $9000
UART_IVEC equ 16

	org 0
	jp main

	org $38
rst7:	jp intr

	org $80
main:
	ld sp, $0
	im 2

	ld a, IVT_BASE >> 8
	ld i, a
	ld ix, IVT_BASE
	ld (ix + UART_IVEC), intr & $ff
	ld (ix + UART_IVEC + 1), intr >> 8

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
	; set the interrupt mask
	ld a, UART_IMASK_RXA
	out (UART_REG_IMASK), a
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

	; set UART interrupt vector
	ld a, UART_IVEC
	out (UART_REG_IVEC), a

	ld bc, INPQ_BASE
	ld (INPQ_HEAD), bc
	ld (INPQ_TAIL), bc

	ei

	ld hl, str_hello
	call uart_putstr

mainloop:
	ld ix, INPQ_HEAD
	ld a, (INPQ_TAIL)
	cp (ix)
	jr z, mainloop
	; we got input
	di
	ld c, (ix)
	ld b, (ix + 1)
	ld a, (bc)
	ex af,af'
	ld a, c
	inc a
	and $f
	ld (ix), a
	ex af,af'
	ei
	call uart_putchar	; echo
	jr mainloop


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

uart_pending:
	in a, (UART_REG_STATA)
	and 1
	ret

intr:
	exx
	ex af,af'
	call uart_pending
	and a
	jr z, .eoi
	; read from uart and append to input queue
	in a, (UART_REG_DATAA)
	ld hl, (INPQ_TAIL)
	ld (hl), a
	ld a, l
	inc a
	and $f
	ld (INPQ_TAIL), a
	ld hl, INPQ_HEAD
	cp (hl)
	jr nz, .eoi
	; overflow
	ld a, (INPQ_HEAD)
	inc a
	and $f
	ld (INPQ_HEAD), a

.eoi:	exx
	ex af,af'
	ei
	ret

str_hello asciiz 'UART interrupt-based echo test (mode 2)',13,10
