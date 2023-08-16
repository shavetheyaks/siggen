; ------------------------------------------------------------------------------
; Bytecode program, to be included with interpreter
; ------------------------------------------------------------------------------


; Pins for Nokia 5110 LCD
.equ	P_LED  = 0x4
.equ	P_SCLK = 0x5
.equ	P_MOSI = 0x3
.equ	P_DC   = 0x0
.equ	P_RST  = 0x1
.equ	P_SCE  = 0x2


.cseg

entry:
	; Set up ports and SPI
	T_EXT	ext_init

	; Reset the LCD
	T_JAL	lcd_reset

	; Do something with the LCD
	T_MOVI	A0, 0x21
	T_JAL	lcd_cmd
	T_MOVI	A0, 0xb8
	T_JAL	lcd_cmd
	T_MOVI	A0, 0x04
	T_JAL	lcd_cmd
	T_MOVI	A0, 0x14
	T_JAL	lcd_cmd
	T_MOVI	A0, 0x20
	T_JAL	lcd_cmd
	T_MOVI	A0, 0x09
	T_JAL	lcd_cmd

	; Idle while blinking the LCD backlight
	T_MOVS	S0, 0
	T_MOVS	S1, 1
led_loop:
	T_DOUT	S1, P_LED
	T_MOVI	A0, 20000
	T_JAL	delay

	T_DOUT	S0, P_LED
	T_MOVI	A0, 20000
	T_JAL	delay

	T_JMP	led_loop


halt:
	T_JMP	halt

delay:
	T_SUBS	A0, 1
	T_BNZ	delay
	T_JALR	TM, RA

lcd_putc:
	T_JALR	TM, RA

lcd_gotoxy:
	T_JALR	TM, RA

lcd_reset:
	T_MOVS	TM, 0
	T_DOUT	TM, P_RST
	T_MOVS	TM, 1
	T_DOUT	TM, P_RST
	T_JALR	TM, RA

lcd_cmd:
	T_MOVI	TM, spi_buf
	T_STB	A0, TM, 0

	T_MOVS	A0, 0
	T_DOUT	A0, P_DC
	T_DOUT	A0, P_SCE
	T_SPI	TM, 1
	T_MOVS	A0, 1
	T_DOUT	A0, P_SCE

	T_JALR	TM, RA

ext_init:
	movw	ZL, r24

	; Ports B and C all outputs
	ldi	r25, 0xff
	out	DDRB, r25
	out	DDRC, r25

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

	ijmp


.dseg

spi_buf:
	.byte	16

