; ------------------------------------------------------------------------------
; siggen.asm
;
; Outputs digital samples in parallel on port D[7:0] pins
; Momentary-close pushbutton on pin C5 cycles through waveforms
; Analog voltage on pin ADC0 selects frequency
; ------------------------------------------------------------------------------

.include "./m328Pdef.inc"

; ------------------------------------------------------------------------------
; Configuration

.equ	STEP_MIN = 25    ; Minimum fractional steps through LUT per cycle
.equ	STEP_MAX = 2560  ; Maximum fractional steps through LUT per cycle


; ------------------------------------------------------------------------------
; Interrupt vector table

.cseg

.org 0x0000
	rjmp	reset     ; RESET
	reti
	reti              ; INT0
	reti
	reti              ; INT1
	reti
	reti              ; PCINT0
	reti
	reti              ; PCINT1
	reti
	reti              ; PCINT2
	reti
	reti              ; WDT
	reti
	reti              ; TIMER2 COMPA
	reti
	reti              ; TIMER2 COMPB
	reti
	reti              ; TIMER2 OVF
	reti
	reti              ; TIMER1 CAPT
	reti
	reti              ; TIMER1 COMPA
	reti
	reti              ; TIMER1 COMPB
	reti
	reti              ; TIMER1 OVF
	reti
	reti              ; TIMER0 COMPA
	reti
	reti              ; TIMER0 COMPB
	reti
	reti              ; TIMER0 OVF
	reti
	reti              ; SPI, STC
	reti
	reti              ; USART, RX
	reti
	reti              ; USART, UDRE
	reti
	reti              ; USART, TX
	reti
	reti              ; ADC
	reti
	reti              ; EE READY
	reti
	reti              ; ANALOG COMP
	reti
	reti              ; TWI
	reti
	reti              ; SPM READY
	reti


; ------------------------------------------------------------------------------
; Registers

; r0
; r1
; r2
; r3
; r4
; r5
; r6
; r7
; r8
; r9
; r10
; r11
; r12
; r13
; r14
; r15
; r16 l\_   Step size within waveform LUT (8.8 fixed point)
; r17 h/
; r18
; r19
; r20
; r21
; r22
; r23 ----- Fractional part of pointer to current sample within waveform table
; r24 ----- Second temporary
; r25 ----- First temporary
; r26 l\_X  (X) Pointer to current sample within waveform table
; r27 h/
; r28 l\_Y
; r29 h/
; r30 l\_Z  (Z) Pointer to next waveform table in rom
; r31 h/


; ------------------------------------------------------------------------------
; Reset entry point

reset:
	; ----- Disable interrupts and set stack pointer to top of SRAM

	eor	r25, r25
	out	SREG, r25
	ldi	r25, HIGH(RAMEND)
	out	SPH, r25
	ldi	r25, LOW(RAMEND)
	out	SPL, r25

	; ----- Set up PORTD for outputting samples, initial output to center of range

	ldi	r25, 0x80
	out	PORTD, r25
	ldi	r25, 0xff
	out	DDRD, r25

	; ----- Set up ADC for reading the frequency knob

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
	ldi	r25, (0 << REFS0) | \
		     (0 << ADLAR) | \
		     (0 << MUX0)
	sts	ADMUX, r25

	; ADCSRB is for the comparator and autotrigger, neither are active

	; ----- Set up button pin and timer for switching waveforms

	; Set PORTC5 as input with pullup
	cbi	DDRC, DDC5
	sbi	PORTC, PORTC5

	; Allow PCINT13 (button) to trigger pin change interrupts
	ldi	r25, (1 << PCINT13)
	sts	PCMSK1, r25

	; Clear any existing pin interrupts
	in	r25, PCIFR
	out	PCIFR, r25

	; No need to set PCCSR bits since we won't actually be getting
	; interrupted, and PCISR bits will be set regardless

	; Hold prescaler in reset so timer remains halted until we need it
	ldi	r25, (1 << TSM) | \
		     (1 << PSRSYNC)
	out	GTCCR, r25

	; Timer 0: CTC mode (reset after compare match)
	;          Clock source CLK_io / 64
	ldi	r25, (1 << WGM01) | \
		     (0 << WGM00)
	out	TCCR0A, r25
	ldi	r25, (0 << WGM02) | \
		     (3 << CS00)
	out	TCCR0B, r25
	sbi	TIFR0, OCF0A  ; Make sure the timer flag is clear
	ldi	r25, 250      ; Count up to 250 (at 16MHz/64 = 250kHz -> 1ms)
	out	OCR0A, r25
	ldi	r25, 0        ; Reset the timer counter
	out	TCNT0, r25

	; Timer prescaler

	; ----- Set up the initial waveform

	; Point Z at the first waveform in the rom
	ldi	ZL, LOW(2*wavetable_begin)
	ldi	ZH, HIGH(2*wavetable_begin)

	; Advance waveforms to load the LUT into ram and reset the cursor
	rcall	next_waveform

	; Initial step size
	ldi	r16, 0x00  ; Fractional part
	ldi	r17, 0x04  ; Integer part

	; --------------------------------------------------------------
	; Main loop
loop:

	; --------------------------------------------------------------
	; ----- Output a sample and advance
	ld	r25, X
	out	PORTD, r25
	add	r23, r16    ; Fractional part
	adc	XL, r17     ; Integer low part (do not carry to high part)
	; --------------------------------------------------------------



	; --------------------------------------------------------------
	; ----- Read the frequency knob and adjust step size

	; Trigger a single conversion
	lds	r25, ADCSRA
	ori	r25, (1 << ADSC)
	sts	ADCSRA, r25

	; Wait for conversion to complete
_adc_wait:
	lds	r25, ADCSRA
	sbrs	r25, ADIF    ; Test for interrupt flag
	rjmp	_adc_wait
	sts	ADCSRA, r25  ; Store back to clear the interrupt flag (w1c)

	; Read value and adjust step size
	lds	r24, ADCL
	lds	r25, ADCH

	; Map V in range (ADC_MIN, ADC_MAX) to (STEP_MIN, STEP_MAX)
	;
	;     /                 (STEP_MAX - STEP_MIN) \
	; X = | (V - ADC_MIN) * --------------------- | + STEP_MIN
	;     \                  (ADC_MAX - ADC_MIN)  /
	;
	; ADC_MIN = 0 and ADC_MAX = 1023 (~1024), so this can be simplified
	;
	; We do V * (STEP_MAX - STEP_MIN) first to avoid losing precision
	; Then divide by (ADC_MAX - ADC_MIN) by shifting right 10 bits
	; (Which we do by dropping the least-significant byte and shifting 2)

	; Get variables ready
	ldi	r19, 0x00
	ldi	r20, LOW(STEP_MAX - STEP_MIN)
	ldi	r21, HIGH(STEP_MAX - STEP_MIN)

	; 16-bit multiplication from 8-bit multiplications
	; Like multiplying two two-digit numbers, but each "digit" is now a byte
	;
	;               r25  r24
	;                 A    B  <- value from ADC
	;               r21  r20
	;            *    C    D  <- scale factor (STEP_MAX - STEP_MIN)
	;           ------------
	;                r7   r6
	;               DBH  DBL
	;           r5   r4
	;          DAH  DAL
	;           r3   r2
	;          CBH  CBL
	;      r1   r0
	;  +  CAH  CAL
	;  ---------------------
	;      r1   r0   r2   r6  <- 32-bit result

	; Generate partial products
	mul	r20, r24
	movw	r6, r0
	mul	r20, r25
	movw	r4, r0
	mul	r21, r24
	movw	r2, r0
	mul	r21, r25

	; Add partial products
	add	r4, r7
	adc	r3, r5
	adc	r1, r19  ; carry only
	add	r2, r4
	adc	r0, r3
	adc	r1, r19  ; carry only
	; Full 32-bit product is in r1:r0:r2:r6

	; Shift down by 10 for division by 1024 (discard r6, shift 2)
	clc
	ror	r1
	ror	r0
	ror	r2
	clc
	ror	r1
	ror	r0
	ror	r2

	; Add offset
	mov	r24, r2
	mov	r25, r0
	adiw	r24, STEP_MIN

	; Copy into step size variable
	movw	r16, r24
	; --------------------------------------------------------------



	; --------------------------------------------------------------
	; ----- Poll for button presses and swap waveforms

	; Test for pin change interrupt flag
	sbis	PCIFR, PCIF1
	rjmp	_button_done
	; Button pin status has changed

	; Advance to next waveform in rom and copy to waveform LUT
	; Do this before the switch debouncing so the time spent copying can be
	; used as part of the wait time for the switch to settle
	rcall	next_waveform

	; Debounce, wait for button release, and debounce again
	rcall	debounce
_button_wait_release:
	sbis	PINC, PINC5
	rjmp	_button_wait_release
	rcall	debounce

_button_done:
	; --------------------------------------------------------------


	rjmp	loop


; ------------------------------------------------------------------------------
; Subroutines


next_waveform:
	; Throughout execution, Z points to the next wavetable to be loaded into
	; the waveform buffer.  This subroutine advances the Z pointer as it
	; copies, and wraps it back around to the beginning of the wavetable if
	; it reaches the end.

	; Copy the next waveform from the wavetable ROM into the waveform buffer
	ldi	XL, LOW(waveform)            ; Point X at the waveform buffer
	ldi	XH, HIGH(waveform)
	ldi	r25, 0x00
_next_waveform_loop:
	lpm	r24, Z+                      ; Load a byte from wavetale
	st	X+, r24                      ; Store it to the waveform buffer
	inc	r25                          ; Advance counter
	brne	_next_waveform_loop          ; Exit when counter overflows to zero

	; Handle wrap-around after cycling through all waveforms
	; Waveforms are all 256 bytes long, so the lower byte of Z will always
	; be the same after copying, so no need to reload it
	cpi	ZH, HIGH(2*wavetable_end)    ; Did we copy the final waveform?
	brne	_next_waveform_nowrap
	ldi	ZH, HIGH(2*wavetable_begin)  ; If so, reset to the first waveform
_next_waveform_nowrap:

	; Reset the current sample pointer
	ldi	r23, 0x00                    ; Fractional part
	ldi	XL, LOW(waveform)            ; Integer low part
	ldi	XH, HIGH(waveform)           ; Integer high part

	ret


debounce:
	; Debouncing is done by waiting for the pin to be quiet (no transitions)
	; for some specified amount of time (1ms for now, see OCR0A in the init
	; section above).
	;
	; We run the timer for 1ms and run in a busyloop until it expires.  If
	; during this loop the pin change flag indicates that there was a
	; transition, we reset the timer and wait again.

_debounce_reset:
	sbi	PCIFR, PCIF1         ; Ensure pin change flag is cleared (w1c)
	sbi	TIFR0, OCF0A         ; Ensure timer expire flag is cleared (w1c)

	; PSRSYNC resets the timer clock prescaler
	; TSM forces the prescaler to be held in reset
	ldi	r25, (1 << TSM) | \
		     (1 << PSRSYNC)
	out	GTCCR, r25           ; Reset the prescaler and halt
	ldi	r25, 0x00
	out	TCNT0, r25           ; Reset timer count value
	out	GTCCR, r25           ; Unhalt the prescaler to run the timer
_debounce_busyloop:
	sbic	PCIFR, PCIF1         ; If the pin changed again, reset and retry
	rjmp	_debounce_reset
	sbis	TIFR0, OCF0A         ; IF the timer expired, exit the loop
	rjmp	_debounce_busyloop

	ret



; ------------------------------------------------------------------------------
; Waveform data

.dseg

; Waveform table must be 256-byte aligned
; (The assembler appears to lack an "align" directive!)
.org	0x100
waveform:
	.byte	256

.cseg

; Waveform data must be contiguous
wavetable_begin:
; Sine
	.db	0x80, 0x83, 0x86, 0x89, 0x8c, 0x8f, 0x92, 0x95
	.db	0x98, 0x9c, 0x9f, 0xa2, 0xa5, 0xa8, 0xab, 0xae
	.db	0xb0, 0xb3, 0xb6, 0xb9, 0xbc, 0xbf, 0xc1, 0xc4
	.db	0xc7, 0xc9, 0xcc, 0xce, 0xd1, 0xd3, 0xd5, 0xd8
	.db	0xda, 0xdc, 0xde, 0xe0, 0xe2, 0xe4, 0xe6, 0xe8
	.db	0xea, 0xec, 0xed, 0xef, 0xf0, 0xf2, 0xf3, 0xf5
	.db	0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfc
	.db	0xfd, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe
	.db	0xfd, 0xfc, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7
	.db	0xf6, 0xf5, 0xf3, 0xf2, 0xf0, 0xef, 0xed, 0xec
	.db	0xea, 0xe8, 0xe6, 0xe4, 0xe2, 0xe0, 0xde, 0xdc
	.db	0xda, 0xd8, 0xd5, 0xd3, 0xd1, 0xce, 0xcc, 0xc9
	.db	0xc7, 0xc4, 0xc1, 0xbf, 0xbc, 0xb9, 0xb6, 0xb3
	.db	0xb0, 0xae, 0xab, 0xa8, 0xa5, 0xa2, 0x9f, 0x9c
	.db	0x98, 0x95, 0x92, 0x8f, 0x8c, 0x89, 0x86, 0x83
	.db	0x80, 0x7c, 0x79, 0x76, 0x73, 0x70, 0x6d, 0x6a
	.db	0x67, 0x63, 0x60, 0x5d, 0x5a, 0x57, 0x54, 0x51
	.db	0x4f, 0x4c, 0x49, 0x46, 0x43, 0x40, 0x3e, 0x3b
	.db	0x38, 0x36, 0x33, 0x31, 0x2e, 0x2c, 0x2a, 0x27
	.db	0x25, 0x23, 0x21, 0x1f, 0x1d, 0x1b, 0x19, 0x17
	.db	0x15, 0x13, 0x12, 0x10, 0x0f, 0x0d, 0x0c, 0x0a
	.db	0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x03
	.db	0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01
	.db	0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
	.db	0x09, 0x0a, 0x0c, 0x0d, 0x0f, 0x10, 0x12, 0x13
	.db	0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 0x21, 0x23
	.db	0x25, 0x27, 0x2a, 0x2c, 0x2e, 0x31, 0x33, 0x36
	.db	0x38, 0x3b, 0x3e, 0x40, 0x43, 0x46, 0x49, 0x4c
	.db	0x4f, 0x51, 0x54, 0x57, 0x5a, 0x5d, 0x60, 0x63
	.db	0x67, 0x6a, 0x6d, 0x70, 0x73, 0x76, 0x79, 0x7c

; Square
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.db	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

; Rising sawtooth
	.db	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
	.db	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	.db	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17
	.db	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
	.db	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
	.db	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f
	.db	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37
	.db	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f
	.db	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47
	.db	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f
	.db	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57
	.db	0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f
	.db	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67
	.db	0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f
	.db	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77
	.db	0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f
	.db	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87
	.db	0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f
	.db	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97
	.db	0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f
	.db	0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7
	.db	0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf
	.db	0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7
	.db	0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf
	.db	0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7
	.db	0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf
	.db	0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7
	.db	0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf
	.db	0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7
	.db	0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef
	.db	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7
	.db	0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff

; Falling sawtooth
	.db	0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8
	.db	0xf7, 0xf6, 0xf5, 0xf4, 0xf3, 0xf2, 0xf1, 0xf0
	.db	0xef, 0xee, 0xed, 0xec, 0xeb, 0xea, 0xe9, 0xe8
	.db	0xe7, 0xe6, 0xe5, 0xe4, 0xe3, 0xe2, 0xe1, 0xe0
	.db	0xdf, 0xde, 0xdd, 0xdc, 0xdb, 0xda, 0xd9, 0xd8
	.db	0xd7, 0xd6, 0xd5, 0xd4, 0xd3, 0xd2, 0xd1, 0xd0
	.db	0xcf, 0xce, 0xcd, 0xcc, 0xcb, 0xca, 0xc9, 0xc8
	.db	0xc7, 0xc6, 0xc5, 0xc4, 0xc3, 0xc2, 0xc1, 0xc0
	.db	0xbf, 0xbe, 0xbd, 0xbc, 0xbb, 0xba, 0xb9, 0xb8
	.db	0xb7, 0xb6, 0xb5, 0xb4, 0xb3, 0xb2, 0xb1, 0xb0
	.db	0xaf, 0xae, 0xad, 0xac, 0xab, 0xaa, 0xa9, 0xa8
	.db	0xa7, 0xa6, 0xa5, 0xa4, 0xa3, 0xa2, 0xa1, 0xa0
	.db	0x9f, 0x9e, 0x9d, 0x9c, 0x9b, 0x9a, 0x99, 0x98
	.db	0x97, 0x96, 0x95, 0x94, 0x93, 0x92, 0x91, 0x90
	.db	0x8f, 0x8e, 0x8d, 0x8c, 0x8b, 0x8a, 0x89, 0x88
	.db	0x87, 0x86, 0x85, 0x84, 0x83, 0x82, 0x81, 0x80
	.db	0x7f, 0x7e, 0x7d, 0x7c, 0x7b, 0x7a, 0x79, 0x78
	.db	0x77, 0x76, 0x75, 0x74, 0x73, 0x72, 0x71, 0x70
	.db	0x6f, 0x6e, 0x6d, 0x6c, 0x6b, 0x6a, 0x69, 0x68
	.db	0x67, 0x66, 0x65, 0x64, 0x63, 0x62, 0x61, 0x60
	.db	0x5f, 0x5e, 0x5d, 0x5c, 0x5b, 0x5a, 0x59, 0x58
	.db	0x57, 0x56, 0x55, 0x54, 0x53, 0x52, 0x51, 0x50
	.db	0x4f, 0x4e, 0x4d, 0x4c, 0x4b, 0x4a, 0x49, 0x48
	.db	0x47, 0x46, 0x45, 0x44, 0x43, 0x42, 0x41, 0x40
	.db	0x3f, 0x3e, 0x3d, 0x3c, 0x3b, 0x3a, 0x39, 0x38
	.db	0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30
	.db	0x2f, 0x2e, 0x2d, 0x2c, 0x2b, 0x2a, 0x29, 0x28
	.db	0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20
	.db	0x1f, 0x1e, 0x1d, 0x1c, 0x1b, 0x1a, 0x19, 0x18
	.db	0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x10
	.db	0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08
	.db	0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00

; Triangle
	.db	0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0e
	.db	0x10, 0x12, 0x14, 0x16, 0x18, 0x1a, 0x1c, 0x1e
	.db	0x20, 0x22, 0x24, 0x26, 0x28, 0x2a, 0x2c, 0x2e
	.db	0x30, 0x32, 0x34, 0x36, 0x38, 0x3a, 0x3c, 0x3e
	.db	0x40, 0x42, 0x44, 0x46, 0x48, 0x4a, 0x4c, 0x4e
	.db	0x50, 0x52, 0x54, 0x56, 0x58, 0x5a, 0x5c, 0x5e
	.db	0x60, 0x62, 0x64, 0x66, 0x68, 0x6a, 0x6c, 0x6e
	.db	0x70, 0x72, 0x74, 0x76, 0x78, 0x7a, 0x7c, 0x7e
	.db	0x80, 0x82, 0x84, 0x86, 0x88, 0x8a, 0x8c, 0x8e
	.db	0x90, 0x92, 0x94, 0x96, 0x98, 0x9a, 0x9c, 0x9e
	.db	0xa0, 0xa2, 0xa4, 0xa6, 0xa8, 0xaa, 0xac, 0xae
	.db	0xb0, 0xb2, 0xb4, 0xb6, 0xb8, 0xba, 0xbc, 0xbe
	.db	0xc0, 0xc2, 0xc4, 0xc6, 0xc8, 0xca, 0xcc, 0xce
	.db	0xd0, 0xd2, 0xd4, 0xd6, 0xd8, 0xda, 0xdc, 0xde
	.db	0xe0, 0xe2, 0xe4, 0xe6, 0xe8, 0xea, 0xec, 0xee
	.db	0xf0, 0xf2, 0xf4, 0xf6, 0xf8, 0xfa, 0xfc, 0xfe
	.db	0xff, 0xfd, 0xfb, 0xf9, 0xf7, 0xf5, 0xf3, 0xf1
	.db	0xef, 0xed, 0xeb, 0xe9, 0xe7, 0xe5, 0xe3, 0xe1
	.db	0xdf, 0xdd, 0xdb, 0xd9, 0xd7, 0xd5, 0xd3, 0xd1
	.db	0xcf, 0xcd, 0xcb, 0xc9, 0xc7, 0xc5, 0xc3, 0xc1
	.db	0xbf, 0xbd, 0xbb, 0xb9, 0xb7, 0xb5, 0xb3, 0xb1
	.db	0xaf, 0xad, 0xab, 0xa9, 0xa7, 0xa5, 0xa3, 0xa1
	.db	0x9f, 0x9d, 0x9b, 0x99, 0x97, 0x95, 0x93, 0x91
	.db	0x8f, 0x8d, 0x8b, 0x89, 0x87, 0x85, 0x83, 0x81
	.db	0x7f, 0x7d, 0x7b, 0x79, 0x77, 0x75, 0x73, 0x71
	.db	0x6f, 0x6d, 0x6b, 0x69, 0x67, 0x65, 0x63, 0x61
	.db	0x5f, 0x5d, 0x5b, 0x59, 0x57, 0x55, 0x53, 0x51
	.db	0x4f, 0x4d, 0x4b, 0x49, 0x47, 0x45, 0x43, 0x41
	.db	0x3f, 0x3d, 0x3b, 0x39, 0x37, 0x35, 0x33, 0x31
	.db	0x2f, 0x2d, 0x2b, 0x29, 0x27, 0x25, 0x23, 0x21
	.db	0x1f, 0x1d, 0x1b, 0x19, 0x17, 0x15, 0x13, 0x11
	.db	0x0f, 0x0d, 0x0b, 0x09, 0x07, 0x05, 0x03, 0x01

; White noise, contains all values 0-255, randomized
	.db	0x7a, 0x1b, 0x50, 0xac, 0xa6, 0xf7, 0xf1, 0x82
	.db	0x6d, 0xfc, 0x24, 0x5d, 0x6e, 0xe9, 0xd6, 0x74
	.db	0x08, 0x8e, 0x28, 0xf6, 0x13, 0x22, 0xd3, 0xb1
	.db	0x60, 0x5c, 0x67, 0x21, 0xca, 0x01, 0x72, 0x2f
	.db	0xd8, 0xeb, 0x32, 0x3a, 0x88, 0x81, 0x83, 0x59
	.db	0x7d, 0x5b, 0x2a, 0x87, 0x5f, 0x99, 0x96, 0x09
	.db	0xef, 0xd5, 0x2c, 0x17, 0x94, 0x8c, 0xff, 0xdd
	.db	0x57, 0xe5, 0x0a, 0xf5, 0x10, 0xaf, 0x36, 0x2e
	.db	0x4b, 0x35, 0xc4, 0xb0, 0xf0, 0x61, 0xab, 0x95
	.db	0x18, 0x26, 0x9e, 0x7f, 0x33, 0x65, 0x9c, 0x70
	.db	0xbf, 0xa7, 0x64, 0xc3, 0x1a, 0x31, 0xee, 0xc8
	.db	0x14, 0x25, 0x76, 0xc6, 0x58, 0x52, 0x03, 0x9b
	.db	0x1f, 0xdf, 0x0f, 0xc1, 0x42, 0x68, 0x27, 0xe0
	.db	0x2b, 0x54, 0xd0, 0xfa, 0x4e, 0x93, 0xa5, 0xf2
	.db	0x05, 0x3b, 0x1d, 0x02, 0xf8, 0xea, 0x45, 0x4a
	.db	0x04, 0xcb, 0xf9, 0xc5, 0x7c, 0xe4, 0x48, 0x1c
	.db	0xae, 0x75, 0xbe, 0xb4, 0x9f, 0xd9, 0x37, 0x11
	.db	0x46, 0xcd, 0x3f, 0xe2, 0x9a, 0xe1, 0x9d, 0xb2
	.db	0xc0, 0xb3, 0xa3, 0x77, 0x34, 0x80, 0xfd, 0x1e
	.db	0xa1, 0xed, 0x07, 0x6c, 0x30, 0x6a, 0x69, 0x97
	.db	0x23, 0xd7, 0x3e, 0xde, 0x4f, 0x79, 0x8b, 0x4d
	.db	0xc2, 0xb6, 0x84, 0xa4, 0x66, 0xd1, 0xba, 0x0c
	.db	0xf4, 0xbc, 0x8f, 0xd4, 0x40, 0xdb, 0xaa, 0xcf
	.db	0x8a, 0x56, 0xad, 0x7b, 0x0e, 0x3c, 0xc7, 0x71
	.db	0xfb, 0x49, 0x90, 0x98, 0x5e, 0xf3, 0x12, 0x38
	.db	0x00, 0xe3, 0x0b, 0xec, 0x55, 0x39, 0x89, 0xbb
	.db	0x78, 0x91, 0x15, 0x51, 0xe6, 0x47, 0x43, 0x0d
	.db	0xa9, 0xb9, 0x6b, 0x62, 0x73, 0xa0, 0xdc, 0x2d
	.db	0xe8, 0x63, 0xda, 0xd2, 0xfe, 0xc9, 0x29, 0xa2
	.db	0x20, 0x8d, 0x53, 0xa8, 0x19, 0x4c, 0x6f, 0x86
	.db	0xce, 0x92, 0x06, 0xe7, 0x7e, 0xb8, 0x5a, 0x3d
	.db	0xb5, 0x85, 0x44, 0x16, 0xb7, 0xcc, 0x41, 0xbd
wavetable_end:

