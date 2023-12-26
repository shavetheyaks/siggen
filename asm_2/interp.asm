.include "./m328Pdef.inc"
.include "./util.inc"
.include "./tbc.inc"


; ------------------------------------------------------------------------------
; Registers

; r0  ----- Temp (r0:r1 are MUL output, so can't use long-term)
; r1  ----- Temp
; r2  l\___ Interpreter program counter
; r3  h/
; r4  l\___ Current interpreted instruction
; r5  h/
; r6  l\
; r7   |___ Operand A / instruction result
; r8   |
; r9  h/
; r10 l\
; r11  |___ Operand B
; r12  |
; r13 h/
; r14 ----- Flags register value (VF)
; r15 ----- Saved instruction F field
; r16 l\___ Pointer to V[X]
; r17 h/
; r18
; r19
; r20
; r21 ----- Temp
; r22 ----- Temp
; r23 ----- Temp
; r24 ----- Temp
; r25 ----- Temp
; r26 l\_X  HARE reserved
; r27 h/
; r28 l\_Y
; r29 h/
; r30 l\_Z
; r31 h/


; ------------------------------------------------------------------------------
; Locations

.equ	TORTOISE_CODE_START = 0x0000
.equ	TORTOISE_DATA_START = SRAM_START
.equ	IVT_INIT_START      = SMALLBOOTSTART  ; Must be at bootloader start
.equ	INTERPRETER_START   = 0x3800


; ------------------------------------------------------------------------------
; Interrupt vector table and reset code

.cseg

; Put in the bootloader section to leave low memory for bytecode
.org IVT_INIT_START

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


	; ----- Reset entry point
reset:
	; Make sure interrupts are disabled
	clr	r25
	out	SREG, r25

	; Move IVT to the bootloader section
	ldi	r25, (1 << IVCE)
	out	MCUCR, r25
	ldi	r25, (1 << IVSEL)
	out	MCUCR, r25

	; Set stack pointer to top of SRAM
	ldi	r25, HIGH(RAMEND)
	out	SPH, r25
	ldi	r25, LOW(RAMEND)
	out	SPL, r25

	; ----- Set up timer for countdown clocks

	; Hold prescaler in reset so timer remains halted until we need it
	ldi	r25, (1 << TSM) | (1 << PSRSYNC)
	out	GTCCR, r25

	; Timer 0: CTC mode (reset after compare match)
	;          Clock source CLK_io / 1024
	ldi	r25, (0b00 << COM0A0) | (0b00 << COM0B0) | (0b10 << WGM00)
	out	TCCR0A, r25
	ldi	r25, (0b0 << FOC0A) | (0b0 << FOC0B) | (0b0 << WGM02) | (0b101 << CS00)
	out	TCCR0B, r25
	sbi	TIFR0, OCF0A  ; Make sure the timer flag is clear
	ldi	r25, 156      ; Count up to 156 (at 16MHz/1024 = 15.625kHz -> 10ms)
	out	OCR0A, r25
	ldi	r25, 0        ; Reset the timer counter
	out	TCNT0, r25

	; Start the clock
	ldi	r25, (0 << TSM) | (0 << PSRSYNC)
	out	GTCCR, r25

	; ----- Reset interpreter state

	clr	r25

	; Zero out countdown clocks
	ldi	ZL, LOW(interp_clocks)
	ldi	ZH, HIGH(interp_clocks)
	ldi	r24, 16*2
_zclk_loop:
	dec	r24
	brlt	_zclk_done
	st	Z+, r25
	rjmp	_zclk_loop
_zclk_done:

	; Zero out registers
	ldi	ZL, LOW(interp_regs)
	ldi	ZH, HIGH(interp_regs)
	ldi	r24, 16*4
_zreg_loop:
	dec	r24
	brlt	_zreg_done
	st	Z+, r25
	rjmp	_zreg_loop
_zreg_done:

	; Set stack pointer
	ldi	ZL, LOW(interp_regs + (0xd * 4))
	ldi	ZH, HIGH(interp_regs + (0xd * 4))
	ldi	r24, LOW(stack)
	ldi	r25, HIGH(stack)
	st	Z+, r24
	st	Z+, r25

	; Instruction pointer
	ldi	r25, LOW(TORTOISE_CODE_START)
	mov	r2, r25
	ldi	r25, HIGH(TORTOISE_CODE_START)
	mov	r3, r25

	rjmp	loop


; ------------------------------------------------------------------------------
; Main loop

.org INTERPRETER_START
loop:
	; Check for countdown events
	in	r25, TIFR0               ; 1
	sbrs	r25, OCF0A               ; 1/2
	rjmp	_countdown_done          ; 2

	; Clear the timer expire flag
	out	TIFR0, r25               ; 1

	; Decrement each of the countdown clocks
	ldi	ZL, LOW(interp_clocks)   ; 1
	ldi	ZH, HIGH(interp_clocks)  ; 1
	ldi	r23, 16                  ; 1
_countdown_loop:
	dec	r23                      ; 1
	brlt	_countdown_done          ; 1/2*
	ldd	r24, Z+0                 ; 2
	ldd	r25, Z+1                 ; 2
	mov	r22, r24                 ; 1
	or	r22, r25                 ; 1
	breq	_countdown_next          ; 1/2*
	subi	r24, 1                   ; 1
	sbci	r25, 0                   ; 1*
_countdown_next:
	st	Z+, r24                  ; 2
	st	Z+, r25                  ; 2
	rjmp	_countdown_loop          ; 2
_countdown_done:

	; Fetch instruction
	movw	ZL, r2                   ; 1  Put bytecode PC into Z
	lsl	ZL                       ; 1  *2 to get a byte address from a word address
	rol	ZH                       ; 1*
	lpm	r4, Z+                   ; 3  Load a word and increment
	lpm	r5, Z+                   ; 3
	lsr	ZH                       ; 1  /2 to get a word address from a byte address
	ror	ZL                       ; 1*
	movw	r2, ZL                   ; 1  Save updated bytecode PC

	; Decode first operand, always register V[X]
	mov	r24, r5                  ; 1  Extract X field from instruction
	andi	r24, 0x0f                ; 1
	lsl	r24                      ; 1  *4 to get an offset from an index
	lsl	r24                      ; 1
	clr	r25                      ; 1  Add offset to base address of registers
	ldi	ZL, LOW(interp_regs)     ; 1
	ldi	ZH, HIGH(interp_regs)    ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	movw	r16, ZL                  ; 1  Save the reg addr for use as destination later
	ldd	r6, Z+0                  ; 2  Load operand value
	ldd	r7, Z+1                  ; 2
	ldd	r8, Z+2                  ; 2
	ldd	r9, Z+3                  ; 2

	; Decode second operand based on instruction F field
	mov	r24, r5                  ; 1  Extract F field from instruction
	lsr	r24                      ; 1
	lsr	r24                      ; 1
	lsr	r24                      ; 1
	lsr	r24                      ; 1
	clr	r25                      ; 1  Add base address of operand-dispatch jumptable
	ldi	ZL, LOW(operand_jumptable)   ; 1
	ldi	ZH, HIGH(operand_jumptable)  ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	mov	r15, r24                 ; 1  Save the F field for decoding instruction later
	ijmp	                         ; 2  Jump to whatever code decodes the other operand
_decode_done:

	; Load flags value
	ldi	r24, 0x3c                ; 1  Offset of the VF register (0xf * 4)
	clr	r25                      ; 1  Add offset to base address of registers
	ldi	ZL, LOW(interp_regs)     ; 1
	ldi	ZH, HIGH(interp_regs)    ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	ld	r14, Z                   ; 2

	; Dispatch based on instruction F field
	mov	r24, r15                 ; 1  Recover saved F field from instruction
	clr	r25                      ; 1  Add base address of instruction-dispatch jumptable
	ldi	ZL, LOW(f_dispatch_jumptable)   ; 1
	ldi	ZH, HIGH(f_dispatch_jumptable)  ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	ijmp                             ; 2  Jump to whatever code runs this type of instruction
_dispatch_done_writeback_flags:
	; Get rid of the S, N, and Z flags, we're making our own
	ldi	r25, 0xe9                ; 1
	and	r14, r25                 ; 1

	; Compute N flag
	ldi	r25, 0x04                ; 1
	sbrc	r9, 7                    ; 1/2
	or	r14, r25                 ; 1

	; Compute Z flag
	ldi	r24, 0x02                ; 1
	clr	r25                      ; 1
	or	r25, r6                  ; 1
	or	r25, r7                  ; 1
	or	r25, r8                  ; 1
	or	r25, r9                  ; 1
	brne	_no_z                    ; 1/2*
	or	r14, r24                 ; 1
_no_z:

	; Compute S flag
	mov	r25, r14                 ; 1
	lsl	r25                      ; 1  Shift N flag up to where V is
	eor	r25, r14                 ; 1  Xor to get the value for the S flag
	bst	r25, 3                   ; 1  Read flag value
	bld	r14, 4                   ; 1  Write into proper spot

_dispatch_done_writeback_fixedflags:
	ldi	r24, 0x3c                ; 1  Offset of the VF register (0xf * 4)
	clr	r25                      ; 1  Add offset to base address of registers
	ldi	ZL, LOW(interp_regs)     ; 1
	ldi	ZH, HIGH(interp_regs)    ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	st	Z, r14                   ; 2  Store the flag byte generated by the instruction
_dispatch_done_writeback_reg:
	movw	ZL, r16                  ; 1  Recover the pointer to V[X]
	std	Z+0, r6                  ; 2  Save the instruction result to the register
	std	Z+1, r7                  ; 2
	std	Z+2, r8                  ; 2
	std	Z+3, r9                  ; 2
_dispatch_done:

	rjmp	loop                     ; 2


; ------------------------------------------------------------------------------
; Operand decoding

operand_jumptable:
	rjmp	operand_VY               ; 2
	rjmp	operand_imm32            ; 2
	rjmp	operand_Y                ; 2
	rjmp	operand_VY_N             ; 2
	rjmp	operand_VY_N             ; 2
	rjmp	operand_VY_N             ; 2
	rjmp	operand_VY_N             ; 2
	rjmp	operand_VY_N             ; 2
	rjmp	operand_VY_N             ; 2
	rjmp	operand_VY_N             ; 2
	rjmp	operand_VY_N             ; 2
	rjmp	operand_PC_ssNN          ; 2
	rjmp	operand_PC_sNNN          ; 2
	rjmp	operand_PC_sNNN          ; 2
	rjmp	operand_0NNN             ; 2
	rjmp	operand_VY_N             ; 2


; ----- V[Y]
operand_VY:
	mov	r24, r4                  ; 1
	andi	r24, 0xf0                ; 1
	lsr	r24                      ; 1
	lsr	r24                      ; 1
	clr	r25                      ; 1
	ldi	ZL, LOW(interp_regs)     ; 1
	ldi	ZH, HIGH(interp_regs)    ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	ldd	r10, Z+0                 ; 2
	ldd	r11, Z+1                 ; 2
	ldd	r12, Z+2                 ; 2
	ldd	r13, Z+3                 ; 2

	rjmp	_decode_done             ; 2


; ----- 32-bit immediate following instruction
operand_imm32:
	movw	ZL, r2                   ; 1  Put bytecode PC into Z
	lsl	ZL                       ; 1  *2 to get a byte address from a word address
	rol	ZH                       ; 1*
	lpm	r10, Z+                  ; 3  Load four bytes and increment
	lpm	r11, Z+                  ; 3
	lpm	r12, Z+                  ; 3
	lpm	r13, Z+                  ; 3
	lsr	ZH                       ; 1  /2 to get a word address from a byte address
	ror	ZL                       ; 1*
	movw	r2, ZL                   ; 1  Save updated bytecode PC

	rjmp	_decode_done             ; 2


; ----- 4-bit zero-extended immediate within instruction
operand_Y:
	mov	r10, r4                  ; 1
	lsr	r10                      ; 1
	lsr	r10                      ; 1
	lsr	r10                      ; 1
	lsr	r10                      ; 1
	clr	r11                      ; 1
	clr	r12                      ; 1
	clr	r13                      ; 1

	; Most of these instructions have no use for a zero immediate
	; Replace zero with a more useful 0x10 value, for range of 0x01-0x10
	; Instructions that want 0x00-0x0f can mask off the upper nibble
	tst	r10                      ; 1
	brne	_operand_Y_done          ; 1/2*
	ldi	r25, 0x10                ; 1
	mov	r10, r25                 ; 1
_operand_Y_done:

	rjmp	_decode_done             ; 2


; ----- V[Y] + 4-bit zero-extended immediate within instruction
operand_VY_N:
	; Load V[Y]
	mov	r24, r4                  ; 1
	andi	r24, 0xf0                ; 1
	lsr	r24                      ; 1
	lsr	r24                      ; 1
	clr	r25                      ; 1
	ldi	ZL, LOW(interp_regs)     ; 1
	ldi	ZH, HIGH(interp_regs)    ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	ldd	r10, Z+0                 ; 2
	ldd	r11, Z+1                 ; 2
	ldd	r12, Z+2                 ; 2
	ldd	r13, Z+3                 ; 2

	; Add N
	mov	r24, r4                  ; 1
	andi	r24, 0x0f                ; 1
	clr	r25                      ; 1
	add	r10, r24                 ; 1
	adc	r11, r25                 ; 1*
	adc	r12, r25                 ; 1*
	adc	r13, r25                 ; 1*

	rjmp	_decode_done             ; 2


; ----- Zero-extended 12-bit immediate within instruction
operand_0NNN:
	movw	r10, r4                  ; 1
	ldi	r25, 0x0f                ; 1
	and	r11, r25                 ; 1
	clr	r12                      ; 1
	clr	r13                      ; 1
	rjmp	_decode_done             ; 2


; ----- PC + sign-extended 8-bit immediate within instruction
operand_PC_ssNN:
	; Sign-extend 8-bit immediate
	mov	r10, r4                  ; 1
	clr	r11                      ; 1
	clr	r12                      ; 1
	clr	r13                      ; 1
	sbrs	r10, 7                   ; 1/2
	rjmp	_sext_done_PCssNN        ; 2
	com	r11                      ; 1
	com	r12                      ; 1
	com	r13                      ; 1
_sext_done_PCssNN:

	; Add PC
	clr	r25                      ; 1
	add	r10, r2                  ; 1
	adc	r11, r3                  ; 1*
	adc	r12, r25                 ; 1*
	adc	r13, r25                 ; 1*

	rjmp	_decode_done             ; 2


; ----- PC + sign-extended 12-bit immediate within instruction
operand_PC_sNNN:
	; Sign-extend 12-bit immediate
	movw	r10, r4                  ; 1
	ldi	r25, 0x0f                ; 1
	and	r11, r25                 ; 1
	clr	r12                      ; 1
	clr	r13                      ; 1
	sbrs	r11, 3                   ; 1/2
	rjmp	_sext_done_PCsNNN        ; 2
	ldi	r25, 0xf0                ; 1
	or	r11, r25                 ; 1
	com	r12                      ; 1
	com	r13                      ; 1
_sext_done_PCsNNN:

	; Add PC
	clr	r25                      ; 1
	add	r10, r2                  ; 1
	adc	r11, r3                  ; 1*
	adc	r12, r25                 ; 1*
	adc	r13, r25                 ; 1*

	rjmp	_decode_done             ; 2


operand_none:
	rjmp	_decode_done             ; 2


; ------------------------------------------------------------------------------
; Instruction dispatch

f_dispatch_jumptable:
	rjmp	dispatch_alu             ; 2
	rjmp	dispatch_alu             ; 2
	rjmp	dispatch_imm4            ; 2
	rjmp	exec_ldb                 ; 2
	rjmp	exec_ldh                 ; 2
	rjmp	exec_ldw                 ; 2
	rjmp	exec_stb                 ; 2
	rjmp	exec_sth                 ; 2
	rjmp	exec_stw                 ; 2
	rjmp	exec_lpb                 ; 2
	rjmp	exec_lph                 ; 2
	rjmp	dispatch_branch          ; 2
	rjmp	exec_jal_with_ve         ; 2
	rjmp	exec_jmp                 ; 2
	rjmp	exec_ext                 ; 2
	rjmp	exec_lpw                 ; 2

alu_dispatch_jumptable:
	rjmp	exec_add                 ; 2
	rjmp	exec_sub                 ; 2
	rjmp	exec_and                 ; 2
	rjmp	exec_or                  ; 2
	rjmp	exec_xor                 ; 2
	rjmp	exec_nor                 ; 2
	rjmp	exec_mov                 ; 2
	rjmp	exec_mul                 ; 2
	rjmp	exec_test                ; 2
	rjmp	exec_cmp                 ; 2
	rjmp	exec_udiv                ; 2
	rjmp	exec_umod                ; 2
	rjmp	exec_sdiv                ; 2
	rjmp	exec_smod                ; 2
	rjmp	exec_nop                 ; 2
	rjmp	exec_jal                 ; 2

imm4_dispatch_jumptable:
	rjmp	exec_add                 ; 2
	rjmp	exec_sub                 ; 2
	rjmp	exec_mov                 ; 2
	rjmp	exec_shl                 ; 2
	rjmp	exec_shrl                ; 2
	rjmp	exec_shra                ; 2
	rjmp	exec_rol                 ; 2
	rjmp	exec_ror                 ; 2
	rjmp	exec_spi                 ; 2
	rjmp	exec_mft                 ; 2
	rjmp	exec_mtt                 ; 2
	rjmp	exec_ddir                ; 2
	rjmp	exec_din                 ; 2
	rjmp	exec_dout                ; 2
	rjmp	exec_ain                 ; 2
	rjmp	exec_aout                ; 2

branch_dispatch_jumptable:
	rjmp	exec_jtab                ; 2
	rjmp	exec_jtab                ; 2
	rjmp	exec_jtab                ; 2
	rjmp	exec_jtab                ; 2
	rjmp	exec_jtab                ; 2
	rjmp	exec_jtab                ; 2
	rjmp	exec_blt                 ; 2
	rjmp	exec_bge                 ; 2
	rjmp	exec_bv                  ; 2
	rjmp	exec_bnv                 ; 2
	rjmp	exec_bmi                 ; 2
	rjmp	exec_bpl                 ; 2
	rjmp	exec_bz                  ; 2
	rjmp	exec_bnz                 ; 2
	rjmp	exec_c                   ; 2
	rjmp	exec_nc                  ; 2


dispatch_alu:
	mov	r24, r4                  ; 1
	andi	r24, 0x0f                ; 1
	clr	r25                      ; 1
	ldi	ZL, LOW(alu_dispatch_jumptable)   ; 1
	ldi	ZH, HIGH(alu_dispatch_jumptable)  ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	ijmp                             ; 2

dispatch_imm4:
	mov	r24, r4                  ; 1
	andi	r24, 0x0f                ; 1
	clr	r25                      ; 1
	ldi	ZL, LOW(imm4_dispatch_jumptable)   ; 1
	ldi	ZH, HIGH(imm4_dispatch_jumptable)  ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	ijmp                             ; 2

dispatch_branch:
	mov	r24, r5                  ; 1
	andi	r24, 0x0f                ; 1
	clr	r25                      ; 1
	ldi	ZL, LOW(branch_dispatch_jumptable)   ; 1
	ldi	ZH, HIGH(branch_dispatch_jumptable)  ; 1
	add	ZL, r24                  ; 1
	adc	ZH, r25                  ; 1*
	ijmp                             ; 2


exec_nop:
	rjmp	_dispatch_done           ; 2


exec_add:
	add	r6, r10                  ; 1
	adc	r7, r11                  ; 1*
	adc	r8, r12                  ; 1*
	adc	r9, r13                  ; 1*

	; Flags
	in	r24, SREG                ; 1* Load real flags and keep xxxSVxxC
	andi	r24, 0x19                ; 1
	ldi	r25, 0xe6                ; 1  Clear old flag bits that we're taking from SREG
	and	r14, r25                 ; 1
	or	r14, r24                 ; 1  Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_sub:
	sub	r6, r10                  ; 1
	sbc	r7, r11                  ; 1*
	sbc	r8, r12                  ; 1*
	sbc	r9, r13                  ; 1*

	; Flags
	in	r24, SREG                ; 1* Load real flags and keep xxxSVxxC
	andi	r24, 0x19                ; 1
	ldi	r25, 0xe6                ; 1  Clear old flag bits that we're taking from SREG
	and	r14, r25                 ; 1
	or	r14, r24                 ; 1  Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_and:
	and	r6, r10                  ; 1
	and	r7, r11                  ; 1
	and	r8, r12                  ; 1
	and	r9, r13                  ; 1

	; Flags
	in	r24, SREG                ; 1* Load real flags and keep xxxSVxxx
	andi	r24, 0x18                ; 1
	ldi	r25, 0xe7                ; 1  Clear old flag bits that we're taking from SREG
	and	r14, r25                 ; 1
	or	r14, r24                 ; 1  Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_or:
	or	r6, r10                  ; 1
	or	r7, r11                  ; 1
	or	r8, r12                  ; 1
	or	r9, r13                  ; 1

	; Flags
	in	r24, SREG                ; 1* Load real flags and keep xxxSVxxx
	andi	r24, 0x18                ; 1
	ldi	r25, 0xe7                ; 1  Clear old flag bits that we're taking from SREG
	and	r14, r25                 ; 1
	or	r14, r24                 ; 1  Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_xor:
	eor	r6, r10                  ; 1
	eor	r7, r11                  ; 1
	eor	r8, r12                  ; 1
	eor	r9, r13                  ; 1

	; Flags
	in	r24, SREG                ; 1* Load real flags and keep xxxSVxxx
	andi	r24, 0x18                ; 1
	ldi	r25, 0xe7                ; 1  Clear old flag bits that we're taking from SREG
	and	r14, r25                 ; 1
	or	r14, r24                 ; 1  Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_nor:
	or	r6, r10                  ; 1
	or	r7, r11                  ; 1
	or	r8, r12                  ; 1
	or	r9, r13                  ; 1
	com	r6                       ; 1
	com	r7                       ; 1
	com	r8                       ; 1
	com	r9                       ; 1

	; Flags
	in	r24, SREG                ; 1* Load real flags and keep xxxSVxxx
	andi	r24, 0x18                ; 1
	ldi	r25, 0xe7                ; 1  Clear old flag bits that we're taking from SREG
	and	r14, r25                 ; 1
	or	r14, r24                 ; 1  Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_mov:
	movw	r6, r10                  ; 1
	movw	r8, r12                  ; 1
	rjmp	_dispatch_done_writeback_reg  ; 2


exec_mul:
	clr	r0                       ; 1  Zero for adding carries
	clr	r1                       ; 1  Carry accumulation
	ldi	r21, 32                  ; 1  Loop counter
	clr	r22                      ; 1  Temporary for result
	clr	r23                      ; 1
	clr	r24                      ; 1
	clr	r25                      ; 1

	; Multiply
_mul_loop:
	dec	r21                      ; 1
	brmi	_mul_done                ; 1/2*

	lsl	r22                      ; 1  Shift result one bit up
	rol	r23                      ; 1*
	rol	r24                      ; 1*
	rol	r25                      ; 1*
	adc	r1, r0                   ; 1*

	lsl	r10                      ; 1  Shift multiplier one bit up
	rol	r11                      ; 1*
	rol	r12                      ; 1*
	rol	r13                      ; 1*

	brcc	_mul_loop                ; 1/2* If the multiplier high bit was 1, add multiplicand
	add	r22, r6                  ; 1
	adc	r23, r7                  ; 1*
	adc	r24, r8                  ; 1*
	adc	r25, r9                  ; 1*
	adc	r1, r0                   ; 1*
	rjmp	_mul_loop                ; 2
_mul_done:

	; Copy low half of temporary to result (frees up temp regs for flags)
	movw	r6, r22                  ; 1

	mov	r22, r14                 ; 1  Copy flags to temp
	andi	r22, 0xf6                ; 1  Clear V, and C flags

	; Set carry flag if any of the upper 32 bits of result would be set
	tst	r1                       ; 1
	breq	_mul_no_carry            ; 1/2*
	ori	r22, 0x01                ; 1
_mul_no_carry:

	; Set overflow flag if sign of result disagrees with signs of inputs
	mov	r23, r9                  ; 1
	eor	r23, r13                 ; 1  Top bit of r23 is one if result should be negative
	eor	r23, r25                 ; 1  Top bit of r23 is one if result sign is incorrect
	sbrc	r23, 7                   ; 1/2
	ori	r22, 0x08                ; 1

	mov	r14, r22                 ; 1  Copy temp back into flags

	; Copy high half of temporary to result
	movw	r8, r24                  ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_udiv:
	mov	r1, r9                   ; 1
	eor	r1, r13                  ; 1  Bit 7 is set if the result needs to be negated
	clr	r0                       ; 1  For adding carries

	ldi	r25, 0xf6                ; 1  Discard overflow and carry flags
	and	r14, r25                 ; 1

	; Set carry flag if dividing by zero
	; Then divide anyway, because it's hilarious
	mov	r25, r10                 ; 1
	or	r25, r11                 ; 1
	or	r25, r12                 ; 1
	or	r25, r13                 ; 1
	brne	_udiv_no_divz            ; 1/2*
	ldi	r25, 0x01                ; 1
	or	r14, r25                 ; 1
_udiv_no_divz:

	; Call/ret take more than three clock cycles, so they can't be used
	ldi	ZL, LOW(_div_done)       ; 1
	ldi	ZH, HIGH(_div_done)      ; 1
	rjmp	div_subroutine           ; 2
_div_done:

	; Set the overflow flag if the sign is unexpected
	ldi	r25, 0x08                ; 1
	eor	r1, r9                   ; 1
	sbrc	r1, 7                    ; 1/2
	or	r14, r25                 ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_umod:
	mov	r1, r13                  ; 1  Bit 7 is set if the result needs to be negated
	clr	r0                       ; 1  For adding carries

	ldi	r25, 0xf6                ; 1  Discard overflow and carry flags
	and	r14, r25                 ; 1

	; Set carry flag if dividing by zero
	; Then divide anyway, because it's hilarious
	mov	r25, r10                 ; 1
	or	r25, r11                 ; 1
	or	r25, r12                 ; 1
	or	r25, r13                 ; 1
	brne	_umod_no_divz            ; 1/2*
	ldi	r25, 0x01                ; 1
	or	r14, r25                 ; 1
_umod_no_divz:

	; Call/ret take more than three clock cycles, so they can't be used
	ldi	ZL, LOW(_mod_done)       ; 1
	ldi	ZH, HIGH(_mod_done)      ; 1
	rjmp	div_subroutine           ; 2
_mod_done:

	movw	r6, r22                  ; 1
	movw	r8, r24                  ; 1

	; Set the overflow flag if the sign is unexpected
	ldi	r25, 0x08                ; 1
	eor	r1, r9                   ; 1
	sbrc	r1, 7                    ; 1/2
	or	r14, r25                 ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_sdiv:
	mov	r1, r9                   ; 1
	eor	r1, r13                  ; 1  Bit 7 is set if the result needs to be negated
	clr	r0                       ; 1
	dec	r0                       ; 1  For adding carries during inversion

	ldi	r25, 0xf6                ; 1  Discard overflow and carry flags
	and	r14, r25                 ; 1

	; Set carry flag if dividing by zero
	; Then divide anyway, because it's hilarious
	mov	r25, r10                 ; 1
	or	r25, r11                 ; 1
	or	r25, r12                 ; 1
	or	r25, r13                 ; 1
	brne	_sdiv_no_divz            ; 1/2*
	ldi	r25, 0x01                ; 1
	or	r14, r25                 ; 1
_sdiv_no_divz:

	; Absolute value of dividend
	bst	r9, 7                    ; 1
	brtc	_sdiv_no_inv_a           ; 1/2*
	com	r9                       ; 1
	com	r8                       ; 1
	com	r7                       ; 1
	neg	r6                       ; 1
	sbc	r7, r0                   ; 1*
	sbc	r8, r0                   ; 1*
	sbc	r9, r0                   ; 1*
_sdiv_no_inv_a:

	; Absolute value of divisor
	bst	r13, 7                   ; 1
	brtc	_sdiv_no_inv_b           ; 1/2*
	com	r13                      ; 1
	com	r12                      ; 1
	com	r11                      ; 1
	neg	r10                      ; 1
	sbc	r11, r0                  ; 1*
	sbc	r12, r0                  ; 1*
	sbc	r13, r0                  ; 1*
_sdiv_no_inv_b:

	; Call/ret take more than three clock cycles, so they can't be used
	ldi	ZL, LOW(_sdiv_done)      ; 1
	ldi	ZH, HIGH(_sdiv_done)     ; 1
	rjmp	div_subroutine           ; 2
_sdiv_done:

	; Invert result if necessary
	bst	r1, 7                    ; 1
	brtc	_sdiv_no_inv             ; 1/2*
	com	r6                       ; 1
	com	r7                       ; 1
	com	r8                       ; 1
	com	r9                       ; 1
	inc	r6                       ; 1
	adc	r7, r0                   ; 1*
	adc	r8, r0                   ; 1*
	adc	r9, r0                   ; 1*
_sdiv_no_inv:

	; Set the overflow flag if the sign is unexpected
	ldi	r25, 0x08                ; 1
	eor	r1, r9                   ; 1
	sbrc	r1, 7                    ; 1/2
	or	r14, r25                 ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_smod:
	mov	r1, r9                   ; 1
	eor	r1, r13                  ; 1  Bit 7 is set if the result is negative
	bst	r13, 7                   ; 1
	bld	r1, 6                    ; 1  Bit 6 is set if the modulo needs to be negated
	clr	r0                       ; 1
	dec	r0                       ; 1  For adding carries during inversion

	ldi	r25, 0xf6                ; 1  Discard overflow and carry flags
	and	r14, r25                 ; 1

	; Set carry flag if dividing by zero
	; Then divide anyway, because it's hilarious
	mov	r25, r10                 ; 1
	or	r25, r11                 ; 1
	or	r25, r12                 ; 1
	or	r25, r13                 ; 1
	brne	_smod_no_divz            ; 1/2*
	ldi	r25, 0x01                ; 1
	or	r14, r25                 ; 1
_smod_no_divz:

	; Absolute value of dividend
	bst	r9, 7                    ; 1
	brtc	_smod_no_inv_a           ; 1/2*
	com	r9                       ; 1
	com	r8                       ; 1
	com	r7                       ; 1
	neg	r6                       ; 1
	sbc	r7, r0                   ; 1*
	sbc	r8, r0                   ; 1*
	sbc	r9, r0                   ; 1*
_smod_no_inv_a:

	; Absolute value of divisor
	bst	r13, 7                   ; 1
	brtc	_smod_no_inv_b           ; 1/2*
	com	r13                      ; 1
	com	r12                      ; 1
	com	r11                      ; 1
	neg	r10                      ; 1
	sbc	r11, r0                  ; 1*
	sbc	r12, r0                  ; 1*
	sbc	r13, r0                  ; 1*
_smod_no_inv_b:

	; Call/ret take more than three clock cycles, so they can't be used
	ldi	ZL, LOW(_smod_done)      ; 1
	ldi	ZH, HIGH(_smod_done)     ; 1
	rjmp	div_subroutine           ; 2
_smod_done:

	; Adjust modulo if division result is negative
	bst	r1, 7                    ; 1
	brtc	_smod_no_adj             ; 1/2*
	sub	r10, r22                 ; 1
	sbc	r11, r23                 ; 1*
	sbc	r12, r24                 ; 1*
	sbc	r13, r25                 ; 1*
	; Invert modulo if divisor was negative
	bst	r1, 6                    ; 1
	brtc	_smod_no_inv             ; 1/2*
	com	r13                      ; 1
	com	r12                      ; 1
	com	r11                      ; 1
	neg	r10                      ; 1
	sbc	r11, r0                  ; 1*
	sbc	r12, r0                  ; 1*
	sbc	r13, r0                  ; 1*
_smod_no_inv:
	; Copy adjusted modulo
	movw	r6, r10                  ; 1
	movw	r8, r12                  ; 1
	rjmp	_smod_doflags            ; 2
_smod_no_adj:
	; Non-negative, copy modulo as-is
	movw	r6, r22                  ; 1
	movw	r8, r24                  ; 1
_smod_doflags:

	; Set the overflow flag if the sign is unexpected
	ldi	r25, 0x08                ; 1
	eor	r1, r9                   ; 1
	sbrc	r1, 7                    ; 1/2
	or	r14, r25                 ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


div_subroutine:
	; r22:r23:r24:r25 - remainder
	; r6 :r7 :r8 :r9  - dividend/result
	; r10:r11:r12:r13 - divisor
	; r21             - loop counter

	clr	r22                      ; 1
	clr	r23                      ; 1
	clr	r24                      ; 1
	clr	r25                      ; 1
	ldi	r21, 32                  ; 1
_div_loop:
	lsl	r6                       ; 1
	rol	r7                       ; 1*
	rol	r8                       ; 1*
	rol	r9                       ; 1*
	rol	r22                      ; 1*
	rol	r23                      ; 1*
	rol	r24                      ; 1*
	rol	r25                      ; 1*

	cp	r22, r10                 ; 1
	cpc	r23, r11                 ; 1*
	cpc	r24, r12                 ; 1*
	cpc	r25, r13                 ; 1*

	brlo	_div_next                ; 1/2*
	sub	r22, r10                 ; 1
	sbc	r23, r11                 ; 1*
	sbc	r24, r12                 ; 1*
	sbc	r25, r13                 ; 1*
	inc	r6                       ; 1
_div_next:

	dec	r21                      ; 1
	brne	_div_loop                ; 1/2*

	ijmp                             ; 2


exec_cmp:
	movw	r22, r6                  ; 1
	movw	r24, r8                  ; 1
	sub	r22, r10                 ; 1
	sbc	r23, r11                 ; 1*
	sbc	r24, r12                 ; 1*
	sbc	r25, r13                 ; 1*
	in	r21, SREG                ; 1*
	andi	r21, 0x1d                ; 1

	or	r25, r24                 ; 1
	or	r25, r23                 ; 1
	or	r25, r22                 ; 1
	brne	_cmp_nz                  ; 1/2*
	ori	r21, 0x02                ; 1
_cmp_nz:
	ldi	r25, 0xe0                ; 1
	and	r14, r25                 ; 1
	or	r14, r21                 ; 1
	rjmp	_dispatch_done_writeback_fixedflags  ; 2


exec_test:
	clr	r0                       ; 1
	mov	r25, r14                 ; 1
	andi	r25, 0xf9                ; 1

	mov	r24, r6                  ; 1
	and	r24, r10                 ; 1
	or	r0, r24                  ; 1

	mov	r24, r7                  ; 1
	and	r24, r11                 ; 1
	or	r0, r24                  ; 1

	mov	r24, r8                  ; 1
	and	r24, r12                 ; 1
	or	r0, r24                  ; 1

	mov	r24, r9                  ; 1
	and	r24, r13                 ; 1
	sbrc	r24, 7                   ; 1/2
	ori	r25, 0x04                ; 1
	or	r0, r24                  ; 1

	breq	_test_z                  ; 1/2*
	ori	r25, 0x02                ; 1
_test_z:
	mov	r14, r25                 ; 1
	rjmp	_dispatch_done_writeback_fixedflags  ; 2


exec_jal_with_ve:
	; Change destination pointer to V[E]
	ldi	r24, 0x0e*4              ; 1
	clr	r25                      ; 1
	ldi	r16, LOW(interp_regs)    ; 1
	ldi	r17, HIGH(interp_regs)   ; 1
	add	r16, r24                 ; 1
	adc	r17, r25                 ; 1*
	; Fall-through to normal jump-and-link code
exec_jal:
	movw	r6, r2                   ; 1
	clr	r8                       ; 1
	clr	r9                       ; 1
	movw	r2, r10                  ; 1
	rjmp	_dispatch_done_writeback_reg  ; 2


exec_shl:
	clr	r24                      ; 1  Zero for adding carries
	clr	r25                      ; 1  To accumulate carries
	mov	r1, r9                   ; 1  For overflow flag

_shl_loop:
	dec	r10                      ; 1  Decrement counter
	brlt	_shl_done                ; 1/2*

	lsl	r6                       ; 1  Shift left by a bit
	rol	r7                       ; 1*
	rol	r8                       ; 1*
	rol	r9                       ; 1*
	adc	r25, r24                 ; 1* Accumulate carries

	rjmp	_shl_loop                ; 2
_shl_done:

	mov	r24, r14                 ; 1  Discard overflow and carry flags
	andi	r24, 0xf6                ; 1

	; Set carry flag if any bits were shifted out
	tst	r25                      ; 1
	breq	_shl_no_carry            ; 1/2*
	ori	r24, 0x01                ; 1
_shl_no_carry:

	; Set overflow flag if sign changed
	eor	r1, r9                   ; 1
	sbrc	r1, 7                    ; 1/2
	ori	r24, 0x04                ; 1

	mov	r14, r24                 ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_shrl:
	clr	r24                      ; 1  Zero for adding carries
	clr	r25                      ; 1  To accumulate carries
	mov	r1, r9                   ; 1  For overflow flag

_shrl_loop:
	dec	r10                      ; 1  Decrement counter
	brlt	_shrl_done               ; 1/2*

	lsr	r9                       ; 1
	ror	r8                       ; 1*
	ror	r7                       ; 1*
	ror	r6                       ; 1*
	adc	r25, r24                 ; 1* Accumulate carries

	rjmp	_shrl_loop               ; 2
_shrl_done:

	mov	r24, r14                 ; 1  Discard overflow and carry flags
	andi	r24, 0xf6                ; 1

	; Set carry flag if any bits were shifted out
	tst	r25                      ; 1
	breq	_shrl_no_carry           ; 1/2*
	ori	r24, 0x01                ; 1
_shrl_no_carry:

	; Set overflow flag if sign changed
	eor	r1, r9                   ; 1
	sbrc	r1, 7                    ; 1/2
	ori	r24, 0x04                ; 1

	mov	r14, r24                 ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_shra:
	clr	r24                      ; 1  Zero for adding carries
	clr	r25                      ; 1  To accumulate carries

_shra_loop:
	dec	r10                      ; 1  Decrement counter
	brlt	_shra_done               ; 1/2*

	asr	r9                       ; 1
	ror	r8                       ; 1*
	ror	r7                       ; 1*
	ror	r6                       ; 1*
	adc	r25, r24                 ; 1* Accumulate carries

	rjmp	_shra_loop               ; 2
_shra_done:

	mov	r24, r14                 ; 1  Discard overflow and carry flags
	andi	r24, 0xf6                ; 1

	; Set carry flag if any bits were shifted out
	tst	r25                      ; 1
	breq	_shra_no_carry           ; 1/2*
	ori	r24, 0x01                ; 1
_shra_no_carry:

	; Sign will never change, leave overflow flag clear

	mov	r14, r24                 ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_rol:
	mov	r1, r9                   ; 1  For overflow flag

_rol_loop:
	dec	r10                      ; 1  Decrement counter
	brlt	_rol_done                ; 1/2*

	clc                              ; 1  Pull highest bit into carry
	sbrc	r9, 7                    ; 1/2
	sec                              ; 1
	rol	r6                       ; 1*
	rol	r7                       ; 1*
	rol	r8                       ; 1*
	rol	r9                       ; 1*

	rjmp	_rol_loop                ; 2
_rol_done:

	mov	r24, r14                 ; 1  Discard overflow and carry flags
	andi	r24, 0xf6                ; 1

	; No bits will be lost, leave carry flag clear

	; Set overflow flag if sign changed
	eor	r1, r9                   ; 1
	sbrc	r1, 7                    ; 1/2
	ori	r24, 0x04                ; 1

	mov	r14, r24                 ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_ror:
	mov	r1, r9                   ; 1  For overflow flag

_ror_loop:
	dec	r10                      ; 1  Decrement counter
	brlt	_ror_done                ; 1/2*

	clc                              ; 1  Pull lowest bit into carry
	sbrc	r6, 0                    ; 1/2
	sec                              ; 1
	ror	r9                       ; 1*
	ror	r8                       ; 1*
	ror	r7                       ; 1*
	ror	r6                       ; 1*

	rjmp	_ror_loop                ; 2
_ror_done:

	mov	r14, r14                 ; 1  Discard overflow and carry flags
	andi	r24, 0xf6                ; 1

	; No bits will be lost, leave carry flag clear

	; Set overflow flag if sign changed
	eor	r1, r9                   ; 1
	sbrc	r1, 7                    ; 1/2
	ori	r24, 0x04                ; 1

	mov	r14, r24                 ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_spi:
	movw	ZL, r6                   ; 1
_spi_byte_loop:
	dec	r10                      ; 1
	brlt	_spi_done                ; 1/2*
	ld	r25, Z                   ; 2
	out	SPDR, r25                ; 1
_spi_wait_loop:
	in	r25, SPSR                ; 1
	sbrs	r25, SPIF                ; 1/2
	rjmp	_spi_wait_loop           ; 2
	in	r25, SPDR                ; 1
	st	Z+, r25                  ; 2
	rjmp	_spi_byte_loop           ; 2
_spi_done:
	rjmp	_dispatch_done           ; 2


exec_mft:
	; Restrict operand to 0-f
	ldi	r25, 0x0f                ; 1
	and	r10, r25                 ; 1

	lsl	r10                      ; 1
	clr	r25                      ; 1

	ldi	ZL, LOW(interp_clocks)   ; 1
	ldi	ZH, HIGH(interp_clocks)  ; 1
	add	ZL, r10                  ; 1
	adc	ZH, r25                  ; 1*

	ld	r6, Z+                   ; 2
	ld	r7, Z+                   ; 2
	clr	r8                       ; 1
	clr	r9                       ; 1

	rjmp	_dispatch_done_writeback_reg  ; 2


exec_mtt:
	; Restrict operand to 0-f
	ldi	r25, 0x0f                ; 1
	and	r10, r25                 ; 1

	lsl	r10                      ; 1
	clr	r25                      ; 1

	ldi	ZL, LOW(interp_clocks)   ; 1
	ldi	ZH, HIGH(interp_clocks)  ; 1
	add	ZL, r10                  ; 1
	adc	ZH, r25                  ; 1*

	st	Z+, r6                   ; 2
	st	Z+, r7                   ; 2

	rjmp	_dispatch_done           ; 2


exec_ddir:
	; Restrict operand to 0-f
	ldi	r25, 0x0f                ; 1
	and	r10, r25                 ; 1

	; Extract LSB from first operand
	ldi	r23, 0x01                ; 1
	and	r6, r23                  ; 1
	clr	r7                       ; 1

	; Mask of all bits except LSB
	ldi	r24, 0xfe                ; 1
	ldi	r25, 0xff                ; 1

	; Rotate LSB and mask into position specified by second operand
_ddir_loop:
	dec	r10                      ; 1
	brlt	_din_loop_done           ; 1/2*

	sec                              ; 1
	rol	r24                      ; 1*
	rol	r25                      ; 1*

	clc                              ; 1
	rol	r6                       ; 1*
	rol	r7                       ; 1*

	rjmp	_ddir_loop               ; 2
_ddir_loop_done:

	; Read-modify-write
	in	r22, DDRB                ; 1
	in	r23, DDRC                ; 1
	and	r22, r24                 ; 1
	and	r23, r25                 ; 1
	or	r22, r6                  ; 1
	or	r23, r7                  ; 1
	out	DDRB, r22                ; 1
	out	DDRC, r23                ; 1

	rjmp	_dispatch_done           ; 2


exec_din:
	; Restrict operand to 0-f
	ldi	r25, 0x0f                ; 1
	and	r10, r25                 ; 1

	; Read port values
	in	r24, PINB                ; 1
	in	r25, PINC                ; 1

	; Shift desired value into LSB
_din_loop:
	dec	r10                      ; 1
	brlt	_din_loop_done           ; 1/2*

	clc                              ; 1
	ror	r25                      ; 1*
	ror	r24                      ; 1*

	rjmp	_din_loop                ; 2
_din_loop_done:

	; Extract port LSB and put into result LSB
	andi	r24, 0x01                ; 1
	ldi	r25, 0xfe                ; 1
	and	r6, r25                  ; 1
	or	r6, r24                  ; 1

	rjmp	_dispatch_done_writeback_flags  ; 2


exec_dout:
	; Restrict operand to 0-f
	ldi	r25, 0x0f                ; 1
	and	r10, r25                 ; 1

	; Extract LSB from first operand
	ldi	r23, 0x01                ; 1
	and	r6, r23                  ; 1
	clr	r7                       ; 1

	; Mask of all bits except LSB
	ldi	r24, 0xfe                ; 1
	ldi	r25, 0xff                ; 1

	; Rotate LSB and mask into position specified by second operand
_dout_loop:
	dec	r10                      ; 1
	brlt	_dout_loop_done          ; 1/2*

	sec                              ; 1
	rol	r24                      ; 1*
	rol	r25                      ; 1*

	clc                              ; 1
	rol	r6                       ; 1*
	rol	r7                       ; 1*

	rjmp	_dout_loop               ; 2
_dout_loop_done:

	; Read-modify-write
	in	r22, PORTB               ; 1
	in	r23, PORTC               ; 1
	and	r22, r24                 ; 1
	and	r23, r25                 ; 1
	or	r22, r6                  ; 1
	or	r23, r7                  ; 1
	out	PORTB, r22               ; 1
	out	PORTC, r23               ; 1

	rjmp	_dispatch_done           ; 2


exec_ain:
	; Set the ADC source
	lds	r25, ADMUX               ; 2
	andi	r25, 0xf0                ; 1
	mov	r24, r10                 ; 1
	andi	r24, 0x0f                ; 1
	or	r25, r24                 ; 1
	sts	ADMUX, r25               ; 2

	; Trigger a single conversion
	lds	r25, ADCSRA              ; 2
	ori	r25, (1 << ADSC)         ; 1
	sts	ADCSRA, r25              ; 2

	; Wait for conversion to complete
_ain_wait:
	lds	r25, ADCSRA              ; 2
	sbrs	r25, ADIF                ; 1/2
	rjmp	_ain_wait                ; 2
	sts	ADCSRA, r25              ; 2

	; Read value from ADC
	lds	r6, ADCL                 ; 2
	lds	r7, ADCH                 ; 2
	clr	r8                       ; 1
	clr	r9                       ; 1

	rjmp	_dispatch_done_writeback_reg  ; 2


exec_aout:
	; Restrict operand to 0-7
	ldi	r25, 0x07                ; 1
	and	r10, r25                 ; 1

	clr	r25                      ; 1

	ldi	ZL, LOW(_aout_jtab)      ; 1
	ldi	ZH, HIGH(_aout_jtab)     ; 1
	add	ZL, r10                  ; 1
	adc	ZH, r25                  ; 1*
	ijmp                             ; 2

_aout_jtab:
	rjmp	_aout_ocr0a              ; 2
	rjmp	_aout_ocr0b              ; 2
	rjmp	_aout_ocr1a              ; 2
	rjmp	_aout_ocr1b              ; 2
	rjmp	_aout_ocr2a              ; 2
	rjmp	_aout_ocr2b              ; 2
	rjmp	_aout_done               ; 2
	rjmp	_aout_done               ; 2

_aout_ocr0a:
	out	OCR0A, r6                ; 1
	rjmp	_aout_done               ; 2

_aout_ocr0b:
	out	OCR0B, r6                ; 1
	rjmp	_aout_done               ; 2

_aout_ocr1a:
	sts	OCR1AH, r7               ; 2
	sts	OCR1AL, r8               ; 2
	rjmp	_aout_done               ; 2

_aout_ocr1b:
	sts	OCR1BH, r7               ; 2
	sts	OCR1BL, r8               ; 2
	rjmp	_aout_done               ; 2

_aout_ocr2a:
	sts	OCR2A, r6                ; 2
	rjmp	_aout_done               ; 2

_aout_ocr2b:
	sts	OCR2B, r6                ; 2
	rjmp	_aout_done               ; 2

_aout_done:
	rjmp	_dispatch_done           ; 2


exec_ldb:
	; Load byte
	movw	ZL, r10                  ; 1
	ld	r6, Z+                   ; 2

	; Sign extend
	clr	r0                       ; 1
	sbrc	r6, 7                    ; 1/2
	com	r0                       ; 1
	mov	r7, r0                   ; 1
	mov	r8, r0                   ; 1
	mov	r9, r0                   ; 1

	rjmp	_dispatch_done_writeback_reg  ; 2


exec_ldh:
	; Load halfword
	movw	ZL, r10                  ; 1
	ld	r6, Z+                   ; 2
	ld	r7, Z+                   ; 2

	; Sign extend
	clr	r0                       ; 1
	sbrc	r7, 7                    ; 1/2
	com	r0                       ; 1
	mov	r8, r0                   ; 1
	mov	r9, r0                   ; 1

	rjmp	_dispatch_done_writeback_reg  ; 2


exec_ldw:
	; Load word
	movw	ZL, r10                  ; 1
	ld	r6, Z+                   ; 2
	ld	r7, Z+                   ; 2
	ld	r8, Z+                   ; 2
	ld	r9, Z+                   ; 2

	rjmp	_dispatch_done_writeback_reg  ; 2


exec_lpb:
	; Load byte
	movw	ZL, r10                  ; 1
	lpm	r6, Z+                   ; 3

	; Sign extend
	clr	r0                       ; 1
	sbrc	r6, 7                    ; 1/2
	com	r0                       ; 1
	mov	r7, r0                   ; 1
	mov	r8, r0                   ; 1
	mov	r9, r0                   ; 1

	rjmp	_dispatch_done_writeback_reg  ; 2


exec_lph:
	; Load halfword
	movw	ZL, r10                  ; 1
	lpm	r6, Z+                   ; 3
	lpm	r7, Z+                   ; 3

	; Sign extend
	clr	r0                       ; 1
	sbrc	r7, 7                    ; 1/2
	com	r0                       ; 1
	mov	r8, r0                   ; 1
	mov	r9, r0                   ; 1

	rjmp	_dispatch_done_writeback_reg  ; 2


exec_lpw:
	; Load word
	movw	ZL, r10                  ; 1
	lpm	r6, Z+                   ; 3
	lpm	r7, Z+                   ; 3
	lpm	r8, Z+                   ; 3
	lpm	r9, Z+                   ; 3

	rjmp	_dispatch_done_writeback_reg  ; 2


exec_stb:
	movw	ZL, r10                  ; 1
	st	Z+, r6                   ; 2
	rjmp	_dispatch_done           ; 2


exec_sth:
	movw	ZL, r10                  ; 1
	st	Z+, r6                   ; 2
	st	Z+, r7                   ; 2
	rjmp	_dispatch_done           ; 2


exec_stw:
	movw	ZL, r10                  ; 1
	st	Z+, r6                   ; 2
	st	Z+, r7                   ; 2
	st	Z+, r8                   ; 2
	st	Z+, r9                   ; 2
	rjmp	_dispatch_done           ; 2


exec_ext:
	; Can't use regular call/ret instructions, they take more than 3 cycles

	; Put the return address into temporaries
	ldi	r24, LOW(_ext_done)      ; 1
	ldi	r25, HIGH(_ext_done)     ; 1

	; Execute at the target address
	movw	ZL, r10                  ; 1
	ijmp                             ; 2
_ext_done:
	rjmp	_dispatch_done           ; 2


exec_jtab:
	add	r10, r6                  ; 1  Add V[X] to PC+sext(nn)
	adc	r11, r7                  ; 1*
	movw	r2, r10                  ; 1
	rjmp	_dispatch_done           ; 2


exec_blt:
	sbrc	r14, 4                   ; 1/2
	movw	r2, r10                  ; 1  Branch if S bit is set
	rjmp	_dispatch_done           ; 2
exec_bge:
	sbrs	r14, 4                   ; 1/2
	movw	r2, r10                  ; 1  Branch if S bit is clear
	rjmp	_dispatch_done           ; 2


exec_bv:
	sbrc	r14, 3                   ; 1/2
	movw	r2, r10                  ; 1  Branch if V bit is set
	rjmp	_dispatch_done           ; 2
exec_bnv:
	sbrs	r14, 3                   ; 1/2
	movw	r2, r10                  ; 1  Branch if V bit is clear
	rjmp	_dispatch_done           ; 2


exec_bmi:
	sbrc	r14, 2                   ; 1/2
	movw	r2, r10                  ; 1  Branch if N bit is set
	rjmp	_dispatch_done           ; 2
exec_bpl:
	sbrs	r14, 2                   ; 1/2
	movw	r2, r10                  ; 1  Branch if N bit is clear
	rjmp	_dispatch_done           ; 2


exec_bz:
	sbrc	r14, 1                   ; 1/2
	movw	r2, r10                  ; 1  Branch if Z bit is set
	rjmp	_dispatch_done           ; 2
exec_bnz:
	sbrs	r14, 1                   ; 1/2
	movw	r2, r10                  ; 1  Branch if Z bit is clear
	rjmp	_dispatch_done           ; 2


exec_c:
	sbrc	r14, 0                   ; 1/2
	movw	r2, r10                  ; 1  Branch if C bit is set
	rjmp	_dispatch_done           ; 2
exec_nc:
	sbrs	r14, 0                   ; 1/2
	movw	r2, r10                  ; 1  Branch if C bit is clear
	rjmp	_dispatch_done           ; 2


exec_jmp:
	movw	r2, r10                  ; 1
	rjmp	_dispatch_done           ; 2


; ------------------------------------------------------------------------------
; Interpreter state

.dseg

; Place data at the top of memory
.org	RAMEND+1 - (16*4) - (16*2) - 16

; Top-of-stack (used by bytecode program) just below interpreter state
stack:
	.byte	16         ; Guard against accidental underflows clobbering regs

; 16 individual countdown timers
interp_clocks:
	.byte	16*2

; 16 general purpose 32-bit "registers" (V0-VF)
interp_regs:
	.byte	16*4

