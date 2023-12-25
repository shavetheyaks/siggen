; ------------------------------------------------------------------------------
; Bytecode program, interpreter included
; ------------------------------------------------------------------------------
.include "interp.asm"


; ------------------------------------------------------------------------------
; Definitions

; Pins for Nokia 5110 LCD
.equ	P_LED  = 0x8
.equ	P_SCLK = 0x5
.equ	P_MOSI = 0x3
.equ	P_DC   = 0x0
.equ	P_RST  = 0x1
.equ	P_SCE  = 0x2

.equ	P_BUTTON = 0x5 + 8


.equ	STEP_MIN = 25
.equ	STEP_MAX = 2560


; ------------------------------------------------------------------------------
; Code section

.cseg
.org TORTOISE_CODE_START

entry:
	; Set up ports and SPI
	T_EXT	ext_init

	; Initialize LCD
	T_JAL	lcd_init

	; Turn on backlight
	T_MOVI	TM, 1
	T_DOUT	TM, P_LED

	T_CLR	S0
	T_MOVS	S1, 1
main_loop:
	; Get ADC value
	T_AIN	S2, 1

	; Print ADC value on line 0
	T_CLR	A0
	T_JAL	lcd_gotoxy
	T_MOV	A0, S2
	T_JAL	lcd_put32x

	; Convert ADC value to step size
	T_MULI	S2, (STEP_MAX - STEP_MIN)
	T_SHRL	S2, 10
	T_ADDI	S2, STEP_MIN

	; Print step size on line 1
	T_CLR	A0
	T_MOVS	A1, 1
	T_JAL	lcd_gotoxy
	T_MOV	A0, S2
	T_JAL	lcd_put32x

	; Convert step size to frequency
	T_MULI	S2, 2000000/256
	T_SHRL	S2, 8

	; Print frequency on line 2
	T_CLR	A0
	T_MOVS	A1, 2
	T_JAL	lcd_gotoxy
	T_MOV	A0, S2
	T_JAL	lcd_put32x

	; Check for pushbutton transitions
	T_DIN	A0, 0xd
	T_CMP	A0, S0
	T_BEQ	_no_transition
	T_ADDS	S1, 1

	T_CMPI	A0, 0
	T_AIN	A1, 1
	T_BNE	_t1
	T_MTT	A1, 0
	T_JMP	_tdone
_t1:
	T_MTT	A1, 1
_tdone:

_no_transition:
	T_MOV	S0, A0

	; Print number of transitions on line 3
	T_CLR	A0
	T_MOVS	A1, 3
	T_JAL	lcd_gotoxy
	T_MOV	A0, S1
	T_JAL	lcd_put32x

	; Print current timer values on lines 4 and 5
	T_CLR	A0
	T_MOVS	A1, 4
	T_JAL	lcd_gotoxy
	T_MFT	A0, 0
	T_JAL	lcd_put32x

	T_CLR	A0
	T_MOVS	A1, 5
	T_JAL	lcd_gotoxy
	T_MFT	A0, 1
	T_JAL	lcd_put32x

	T_JMP	main_loop

halt:
	T_JMP	halt

delay:
	T_SUBS	A0, 1
	T_BNZ	delay
	T_JALR	TM, RA

lcd_put32x:
	T_SUBS	SP, 0xc
	T_STW	RA, SP, 8
	T_STW	S1, SP, 4
	T_STW	S0, SP, 0

	T_MOV	S0, A0
	T_MOVS	S1, 8
_put32x_loop:
	T_ROL	S0, 4
	T_MOV	A0, S0
	T_ANDI	A0, 0x0000000f
	T_JAL	lcd_puthex
	T_SUBS	S1, 1
	T_BNZ	_put32x_loop

	T_LDW	S0, SP, 0
	T_LDW	S1, SP, 4
	T_LDW	RA, SP, 8
	T_ADDS	SP, 0xc
	T_JALR	TM, RA

lcd_puthex:
	T_MULI	A0, 6
	T_MOVI	TM, font<<1
	T_ADD	TM, A0

	T_MOVI	A1, spi_buf

	T_LPW	A2, TM, 0
	T_STW	A2, A1, 0
	T_LPH	A2, TM, 4
	T_STH	A2, A1, 4

	T_CLR	A0
	T_DOUT	A0, P_SCE
	T_MOVS	A0, 1
	T_DOUT	A0, P_DC
	T_SPI	A1, 5
	T_DOUT	A0, P_SCE

	T_JALR	TM, RA

lcd_gotoxy:
	T_SUBS	SP, 8
	T_STW	RA, SP, 4
	T_STW	S0, SP, 0

	T_ANDI	A0, 0x7f
	T_ANDI	A1, 0x07
	T_ORI	A0, 0x80
	T_ORI	A1, 0x40
	T_MOV	S0, A1

	T_JAL	lcd_cmd
	T_MOV	A0, S0
	T_JAL	lcd_cmd

	T_LDW	S0, SP, 0
	T_LDW	RA, SP, 4
	T_ADDS	SP, 8
	T_JALR	TM, RA

lcd_reset:
	T_CLR	TM
	T_DOUT	TM, P_RST
	T_MOVS	TM, 1
	T_DOUT	TM, P_RST
	T_JALR	TM, RA

lcd_init:
	T_SUBS	SP, 4
	T_STW	RA, SP, 0

	T_JAL	lcd_reset

	; 0x21, 0xb8, 0x04, 0x14, 0x20, 0x0c
	T_MOVI	TM, spi_buf
	T_MOVI	A0, 0x1404b821
	T_STW	A0, TM, 0
	T_MOVI	A0, 0x00000c20
	T_STH	A0, TM, 4

	T_CLR	A0
	T_DOUT	A0, P_DC
	T_DOUT	A0, P_SCE

	T_SPI	TM, 6

	T_MOVS	A0, 1
	T_DOUT	A0, P_SCE

	T_JAL	lcd_cls

	T_LDW	RA, SP, 0
	T_ADDS	SP, 4
	T_JALR	TM, RA

lcd_clear_row:
	T_SUBS	SP, 8
	T_STW	RA, SP, 4
	T_STW	S0, SP, 0

	T_ORI	A0, 0x40
	T_JAL	lcd_cmd
	T_MOVI	A0, 0x80
	T_JAL	lcd_cmd

	T_MOVI	S0, 84
_clear_row_loop:
	T_CLR	A0
	T_JAL	lcd_data
	T_SUBS	S0, 1
	T_BNZ	_clear_row_loop

	T_LDW	S0, SP, 0
	T_LDW	RA, SP, 4
	T_ADDS	SP, 8
	T_JALR	TM, RA

lcd_cls:
	T_SUBS	SP, 4
	T_STW	RA, SP, 0

	T_CLR	A0
	T_JAL	lcd_clear_row
	T_MOVS	A0, 1
	T_JAL	lcd_clear_row
	T_MOVS	A0, 2
	T_JAL	lcd_clear_row
	T_MOVS	A0, 3
	T_JAL	lcd_clear_row
	T_MOVS	A0, 4
	T_JAL	lcd_clear_row
	T_MOVS	A0, 5
	T_JAL	lcd_clear_row

	T_LDW	RA, SP, 0
	T_ADDS	SP, 4
	T_JALR	TM, RA

lcd_data:
	T_MOVI	TM, spi_buf
	T_STB	A0, TM, 0

	T_MOVS	A0, 1
	T_DOUT	A0, P_DC
	T_CLR	A0
	T_DOUT	A0, P_SCE
	T_SPI	TM, 1
	T_MOVS	A0, 1
	T_DOUT	A0, P_SCE

	T_JALR	TM, RA

lcd_cmd:
	T_MOVI	TM, spi_buf
	T_STB	A0, TM, 0

	T_CLR	A0
	T_DOUT	A0, P_DC
	T_DOUT	A0, P_SCE
	T_SPI	TM, 1
	T_MOVS	A0, 1
	T_DOUT	A0, P_SCE

	T_JALR	TM, RA

ext_init:
	movw	ZL, r24

	; Port B all outputs
	ldi	r25, 0xff
	out	DDRB, r25

	; Port C
	; 0 - Output for LED
	; 5 - Pullup-input for pushbutton
	ldi	r25, (1 << PINC0)
	out	DDRC, r25
	ldi	r25, (1 << PORTC5)
	out	PORTC, r25

	; Set up SPI
	ldi	r25, (0 << SPIE) | \
		     (1 << SPE)  | \
		     (0 << DORD) | \
		     (1 << MSTR) | \
		     (0 << CPOL) | \
		     (0 << CPHA) | \
		     (0b01 << SPR0)
	out	SPCR, r25
	ldi	r25, (0 << SPI2X)
	out	SPSR, r25

	; Set up ADC
	; ADEN  <= 1  Enable ADC
	; ADSC  <= 0  Do not start conversion
	; ADATE <= 0  Do not autotrigger
	; ADIF  <= 1  Clear any pending interrupt flags (just in case)
	; ADIE  <= 0  Disable ADC interrupts (we will poll ADIF)
	; ADPS  <= 7  System clock 16MHz, ADC requres <= 200kHz, scale by 128
	ldi	r25, (1 << ADEN)  | \
		     (0 << ADSC)  | \
		     (0 << ADATE) | \
		     (1 << ADIF)  | \
		     (0 << ADIE)  | \
		     (7 << ADPS0)
	sts	ADCSRA, r25

	; REFS  <= 0  Use AREF as reference
	; ADLAR <= 0  Do not left-justify results
	; MUX   <= 0  Select input ADC0
	ldi	r25, (0b00 << REFS0) | \
		     (0b0 << ADLAR) | \
		     (0b0000 << MUX0)
	sts	ADMUX, r25

	; Disable digital input for analog input pin
	lds	r25, DIDR0
	ori	r25, (1 << ADC1D)
	sts	DIDR0, r25

	; Set up timer/counter 2 for pwm
	; ldi	r25, ()
	; sts	TCCR2A, r25

	ijmp

font:
	.db	0b01111100, 0b10000010, 0b10000010, 0b01111100, 0b00000000, 0x0
	.db	0b10000100, 0b11111110, 0b10000000, 0b00000000, 0b00000000, 0x1
	.db	0b11000100, 0b10100010, 0b10010010, 0b10001100, 0b00000000, 0x2
	.db	0b10000010, 0b10000010, 0b10010010, 0b01101100, 0b00000000, 0x3
	.db	0b00001110, 0b00010000, 0b00010000, 0b11111110, 0b00000000, 0x4
	.db	0b01001110, 0b10010010, 0b10010010, 0b01100010, 0b00000000, 0x5
	.db	0b01111100, 0b10010010, 0b10010010, 0b01100100, 0b00000000, 0x6
	.db	0b00000010, 0b11100010, 0b00011010, 0b00000110, 0b00000000, 0x7
	.db	0b01101100, 0b10010010, 0b10010010, 0b01101100, 0b00000000, 0x8
	.db	0b01001100, 0b10010010, 0b10010010, 0b01111100, 0b00000000, 0x9
	.db	0b11111100, 0b00010010, 0b00010010, 0b11111100, 0b00000000, 0xa
	.db	0b11111110, 0b10010010, 0b10010010, 0b01101100, 0b00000000, 0xb
	.db	0b00111000, 0b01000100, 0b10000010, 0b10000010, 0b00000000, 0xc
	.db	0b11111110, 0b10000010, 0b01000100, 0b00111000, 0b00000000, 0xd
	.db	0b11111110, 0b10010010, 0b10010010, 0b10000010, 0b00000000, 0xe
	.db	0b11111110, 0b00010010, 0b00010010, 0b00000010, 0b00000000, 0xf


; ------------------------------------------------------------------------------

.dseg
.org TORTOISE_DATA_START

spi_buf:
	.byte	16

