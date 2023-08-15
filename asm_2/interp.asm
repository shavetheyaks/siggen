.include "./m328Pdef.inc"

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
; Interrupt vector table

.cseg

; FIXME: move everything to the bootloader section, let bytecode take low mem
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
; Bytecode

.cseg

entry:
	T_MOVI	V1, 0x11111111
	T_MOV	V2, V1
	T_ADD	V2, V1
	T_OR	V3, V1
	T_OR	V3, V2
	T_MOV	V6, V3
	T_ANDI	V6, 0x0000000f
	T_MUL	V6, V2
	T_MOV	V4, V2
	T_SHL	V4, 4
	T_SHRA	V4, 3
	T_NOR	V4, V4
	T_ADDS	V4, 1
	T_SHL	V4, 3
	T_SHRA	V4, 3
	T_SHL	V4, 3
	T_SHRL	V4, 3
	T_MOVI	V4, -4
	T_CMP	V3, V4
	T_BLT	failure
	T_MOVI	V0, 0xaaaaaaaa
	T_CMP	V3, V4
	T_BAE	failure
	T_MOVI	V0, 0xbbbbbbbb
	T_JAL	func
	T_MOVI	SP, variable
	T_STB	V0, SP, 0
	T_LDB	V0, SP, 0
	T_MOVI	V1, 0xdddddddd
	T_STW	V1, SP, 4
	T_LDH	V1, SP, 4
	T_MOVI	VC, const
	T_LPW	V2, VC, 0
	T_JMP	halt
func:
	T_MOVI	V0, 0xcccccccc
	T_JALR	V1, RA
failure:
	T_MOVI	V0, 0xeeeeeeee
halt:
	T_JMP	halt


const:
	.dw	0xcafe, 0xbeef

.dseg

variable:
	.byte	16


; ------------------------------------------------------------------------------
; Reset entry point

.cseg

reset:
	; ----- Disable interrupts and set stack pointer to top of SRAM

	clr	r25
	out	SREG, r25
	ldi	r25, HIGH(RAMEND)
	out	SPH, r25
	ldi	r25, LOW(RAMEND)
	out	SPL, r25

	; ----- Set up debounce timer

	; Hold prescaler in reset so timer remains halted until we need it
	ldi	r25, (1 << TSM) | (1 << PSRSYNC)
	out	GTCCR, r25

	; Timer 0: CTC mode (reset after compare match)
	;          Clock source CLK_io / 64
	ldi	r25, (1 << WGM01) | (0 << WGM00)
	out	TCCR0A, r25
	ldi	r25, (0 << WGM02) | (3 << CS00)
	out	TCCR0B, r25
	sbi	TIFR0, OCF0A  ; Make sure the timer flag is clear
	ldi	r25, 250      ; Count up to 250 (at 16MHz/64 = 250kHz -> 1ms)
	out	OCR0A, r25
	ldi	r25, 0        ; Reset the timer counter
	out	TCNT0, r25

	; ----- Reset interpreter state

	; Zero out registers
	ldi	ZL, LOW(interp_regs)
	ldi	ZH, HIGH(interp_regs)
	clr	r25
	ldi	r24, 16*4
_zreg_loop:
	dec	r24
	brlt	_zreg_done
	st	Z+, r25
	rjmp	_zreg_loop
_zreg_done:

	; TODO: Set stack pointer

	; Instruction pointer
	ldi	r25, LOW(entry)
	mov	r2, r25
	ldi	r25, HIGH(entry)
	mov	r3, r25

	; --------------------------------------------------------------
	; Main loop
loop:
	; Debounce switches
	in	r25, PCIFR
	tst	r25
	breq	_debounce_done

_debounce_reset:
	; Clear pin change flags
	out	PCIFR, r25

	; Reset the timer
	ldi	r25, (1 << TSM) | (1 << PSRSYNC)
	out	GTCCR, r25
	sbi	TIFR0, OCF0A
	clr	r25
	out	TCNT0, r25
	out	GTCCR, r25

_debounce_busyloop:
	in	r25, PCIFR
	tst	r25
	brne	_debounce_reset
	sbis	TIFR0, OCF0A
	rjmp	_debounce_busyloop
_debounce_done:

	; Fetch instruction
	movw	ZL, r2     ; Put bytecode PC into Z
	lsl	ZL         ; *2 to get a byte address from a word address
	rol	ZH
	lpm	r4, Z+     ; Load a word and increment
	lpm	r5, Z+
	lsr	ZH         ; /2 to get a word address from a byte address
	ror	ZL
	movw	r2, ZL     ; Save updated bytecode PC

	; Decode first operand, always register V[X]
	mov	r24, r5    ; Extract X field from instruction
	andi	r24, 0x0f
	lsl	r24        ; *4 to get an offset from an index
	lsl	r24
	clr	r25        ; Add offset to base address of registers
	ldi	ZL, LOW(interp_regs)
	ldi	ZH, HIGH(interp_regs)
	add	ZL, r24
	adc	ZH, r25
	movw	r16, ZL    ; Save the reg addr for use as destination later
	ldd	r6, Z+0    ; Load operand value
	ldd	r7, Z+1
	ldd	r8, Z+2
	ldd	r9, Z+3

	; Decode second operand based on instruction F field
	mov	r24, r5    ; Extract F field from instruction
	lsr	r24
	lsr	r24
	lsr	r24
	lsr	r24
	clr	r25        ; Add base address of operand-dispatch jumptable
	ldi	ZL, LOW(operand_jumptable)
	ldi	ZH, HIGH(operand_jumptable)
	add	ZL, r24
	adc	ZH, r25
	mov	r15, r24   ; Save the F field for decoding instruction later
	ijmp	           ; Jump to whatever code decodes the other operand
_decode_done:

	; Load flags value
	ldi	r24, 0x3c  ; Offset of the VF register (0xf * 4)
	clr	r25        ; Add offset to base address of registers
	ldi	ZL, LOW(interp_regs)
	ldi	ZH, HIGH(interp_regs)
	add	ZL, r24
	adc	ZH, r25
	ld	r14, Z

	; Dispatch based on instruction F field
	mov	r24, r15   ; Recover saved F field from instruction
	clr	r25        ; Add base address of instruction-dispatch jumptable
	ldi	ZL, LOW(f_dispatch_jumptable)
	ldi	ZH, HIGH(f_dispatch_jumptable)
	add	ZL, r24
	adc	ZH, r25
	ijmp               ; Jump to whatever code runs this type of instruction
_dispatch_done_writeback_flags:
	; Get rid of the N and Z flags, we're making our own
	ldi	r25, 0xf9
	and	r14, r25

	; Compute N flag
	ldi	r25, 0x04
	sbrc	r9, 7
	or	r14, r25

	; Compute Z flag
	ldi	r24, 0x02
	clr	r25
	or	r25, r6
	or	r25, r7
	or	r25, r8
	or	r25, r9
	breq	_no_z
	or	r14, r24
_no_z:

_dispatch_done_writeback_fixedflags:
	ldi	r24, 0x3c  ; Offset of the VF register (0xf * 4)
	clr	r25        ; Add offset to base address of registers
	ldi	ZL, LOW(interp_regs)
	ldi	ZH, HIGH(interp_regs)
	add	ZL, r24
	adc	ZH, r25
	st	Z, r14     ; Store the flag byte generated by the instruction
_dispatch_done_writeback_reg:
	movw	ZL, r16    ; Recover the pointer to V[X]
	std	Z+0, r6    ; Save the instruction result to the register
	std	Z+1, r7
	std	Z+2, r8
	std	Z+3, r9
_dispatch_done:

	rjmp	loop


; ------------------------------------------------------------------------------
; Operand decoding

operand_jumptable:
	rjmp	operand_VY
	rjmp	operand_imm32
	rjmp	operand_Y
	rjmp	operand_VY_N
	rjmp	operand_VY_N
	rjmp	operand_VY_N
	rjmp	operand_VY_N
	rjmp	operand_VY_N
	rjmp	operand_VY_N
	rjmp	operand_VY_N
	rjmp	operand_VY_N
	rjmp	operand_PC_ssNN
	rjmp	operand_PC_sNNN
	rjmp	operand_PC_sNNN
	rjmp	operand_0NNN
	rjmp	operand_VY_N


; ----- V[Y]
operand_VY:
	mov	r24, r4
	andi	r24, 0xf0
	lsr	r24
	lsr	r24
	clr	r25
	ldi	ZL, LOW(interp_regs)
	ldi	ZH, HIGH(interp_regs)
	add	ZL, r24
	adc	ZH, r25
	ldd	r10, Z+0
	ldd	r11, Z+1
	ldd	r12, Z+2
	ldd	r13, Z+3

	rjmp	_decode_done


; ----- 32-bit immediate following instruction
operand_imm32:
	movw	ZL, r2     ; Put bytecode PC into Z
	lsl	ZL         ; *2 to get a byte address from a word address
	rol	ZH
	lpm	r10, Z+    ; Load four bytes and increment
	lpm	r11, Z+
	lpm	r12, Z+
	lpm	r13, Z+
	lsr	ZH         ; /2 to get a word address from a byte address
	ror	ZL
	movw	r2, ZL     ; Save updated bytecode PC

	rjmp	_decode_done


; ----- 4-bit zero-extended immediate within instruction
operand_Y:
	mov	r10, r4
	lsr	r10
	lsr	r10
	lsr	r10
	lsr	r10
	clr	r11
	clr	r12
	clr	r13

	rjmp	_decode_done


; ----- V[Y] + 4-bit zero-extended immediate within instruction
operand_VY_N:
	; Load V[Y]
	mov	r24, r4
	andi	r24, 0xf0
	lsr	r24
	lsr	r24
	clr	r25
	ldi	ZL, LOW(interp_regs)
	ldi	ZH, HIGH(interp_regs)
	add	ZL, r24
	adc	ZH, r25
	ldd	r10, Z+0
	ldd	r11, Z+1
	ldd	r12, Z+2
	ldd	r13, Z+3

	; Add N
	mov	r24, r4
	andi	r24, 0x0f
	clr	r25
	add	r10, r24
	adc	r11, r25
	adc	r12, r25
	adc	r13, r25

	rjmp	_decode_done


; ----- Zero-extended 12-bit immediate within instruction
operand_0NNN:
	movw	r10, r4
	ldi	r25, 0x0f
	and	r11, r25
	clr	r12
	clr	r13
	rjmp	_decode_done


; ----- PC + sign-extended 8-bit immediate within instruction
operand_PC_ssNN:
	; Sign-extend 8-bit immediate
	mov	r10, r4
	clr	r11
	clr	r12
	clr	r13
	sbrs	r10, 7
	rjmp	_sext_done_PCssNN
	com	r11
	com	r12
	com	r13
_sext_done_PCssNN:

	; Add PC
	clr	r25
	add	r10, r2
	adc	r11, r3
	adc	r12, r25
	adc	r13, r25

	rjmp	_decode_done


; ----- PC + sign-extended 12-bit immediate within instruction
operand_PC_sNNN:
	; Sign-extend 12-bit immediate
	movw	r10, r4
	ldi	r25, 0x0f
	and	r11, r25
	clr	r12
	clr	r13
	sbrs	r11, 3
	rjmp	_sext_done_PCsNNN
	ldi	r25, 0xf0
	or	r11, r25
	com	r12
	com	r13
_sext_done_PCsNNN:

	; Add PC
	clr	r25
	add	r10, r2
	adc	r11, r3
	adc	r12, r25
	adc	r13, r25

	rjmp	_decode_done


operand_none:
	rjmp	_decode_done


; ------------------------------------------------------------------------------
; Instruction dispatch

f_dispatch_jumptable:
	rjmp	dispatch_alu
	rjmp	dispatch_alu
	rjmp	dispatch_imm4
	rjmp	exec_ldb
	rjmp	exec_ldh
	rjmp	exec_ldw
	rjmp	exec_stb
	rjmp	exec_sth
	rjmp	exec_stw
	rjmp	exec_lpb
	rjmp	exec_lph
	rjmp	dispatch_branch
	rjmp	exec_jal_with_ve
	rjmp	exec_jmp
	rjmp	exec_ext
	rjmp	exec_lpw

alu_dispatch_jumptable:
	rjmp	exec_add
	rjmp	exec_sub
	rjmp	exec_and
	rjmp	exec_or
	rjmp	exec_xor
	rjmp	exec_nor
	rjmp	exec_mov
	rjmp	exec_mul
	rjmp	exec_div
	rjmp	exec_cmp
	rjmp	exec_test
	rjmp	exec_nop
	rjmp	exec_nop
	rjmp	exec_nop
	rjmp	exec_nop
	rjmp	exec_jal

imm4_dispatch_jumptable:
	rjmp	exec_add
	rjmp	exec_sub
	rjmp	exec_mov
	rjmp	exec_shl
	rjmp	exec_shrl
	rjmp	exec_shra
	rjmp	exec_rol
	rjmp	exec_ror
	rjmp	exec_nop
	rjmp	exec_spi
	rjmp	exec_mft
	rjmp	exec_mtt
	rjmp	exec_din
	rjmp	exec_dout
	rjmp	exec_ain
	rjmp	exec_aout

branch_dispatch_jumptable:
	rjmp	exec_jtab
	rjmp	exec_jtab
	rjmp	exec_jtab
	rjmp	exec_jtab
	rjmp	exec_jtab
	rjmp	exec_jtab
	rjmp	exec_blt
	rjmp	exec_bge
	rjmp	exec_bv
	rjmp	exec_bnv
	rjmp	exec_bmi
	rjmp	exec_bpl
	rjmp	exec_bz
	rjmp	exec_bnz
	rjmp	exec_c
	rjmp	exec_nc


dispatch_alu:
	mov	r24, r4
	andi	r24, 0x0f
	clr	r25
	ldi	ZL, LOW(alu_dispatch_jumptable)
	ldi	ZH, HIGH(alu_dispatch_jumptable)
	add	ZL, r24
	adc	ZH, r25
	ijmp

dispatch_imm4:
	mov	r24, r4
	andi	r24, 0x0f
	clr	r25
	ldi	ZL, LOW(imm4_dispatch_jumptable)
	ldi	ZH, HIGH(imm4_dispatch_jumptable)
	add	ZL, r24
	adc	ZH, r25
	ijmp

dispatch_branch:
	mov	r24, r5
	andi	r24, 0x0f
	clr	r25
	ldi	ZL, LOW(branch_dispatch_jumptable)
	ldi	ZH, HIGH(branch_dispatch_jumptable)
	add	ZL, r24
	adc	ZH, r25
	ijmp


exec_nop:
	rjmp	_dispatch_done


exec_add:
	add	r6, r10
	adc	r7, r11
	adc	r8, r12
	adc	r9, r13

	; Flags
	in	r24, SREG  ; Load real flags and keep xxxSVxxC
	andi	r24, 0x19
	ldi	r25, 0xe6  ; Clear old flag bits that we're taking from SREG
	and	r14, r25
	or	r14, r24   ; Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags


exec_sub:
	sub	r6, r10
	sbc	r7, r11
	sbc	r8, r12
	sbc	r9, r13

	; Flags
	in	r24, SREG  ; Load real flags and keep xxxSVxxC
	andi	r24, 0x19
	ldi	r25, 0xe6  ; Clear old flag bits that we're taking from SREG
	and	r14, r25
	or	r14, r24   ; Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags


exec_and:
	and	r6, r10
	and	r7, r11
	and	r8, r12
	and	r9, r13

	; Flags
	in	r24, SREG  ; Load real flags and keep xxxSVxxx
	andi	r24, 0x18
	ldi	r25, 0xe7  ; Clear old flag bits that we're taking from SREG
	and	r14, r25
	or	r14, r24   ; Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags


exec_or:
	or	r6, r10
	or	r7, r11
	or	r8, r12
	or	r9, r13

	; Flags
	in	r24, SREG  ; Load real flags and keep xxxSVxxx
	andi	r24, 0x18
	ldi	r25, 0xe7  ; Clear old flag bits that we're taking from SREG
	and	r14, r25
	or	r14, r24   ; Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags


exec_xor:
	eor	r6, r10
	eor	r7, r11
	eor	r8, r12
	eor	r9, r13

	; Flags
	in	r24, SREG  ; Load real flags and keep xxxSVxxx
	andi	r24, 0x18
	ldi	r25, 0xe7  ; Clear old flag bits that we're taking from SREG
	and	r14, r25
	or	r14, r24   ; Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags


exec_nor:
	or	r6, r10
	or	r7, r11
	or	r8, r12
	or	r9, r13
	com	r6
	com	r7
	com	r8
	com	r9

	; Flags
	in	r24, SREG  ; Load real flags and keep xxxSVxxx
	andi	r24, 0x18
	ldi	r25, 0xe7  ; Clear old flag bits that we're taking from SREG
	and	r14, r25
	or	r14, r24   ; Add flags from SREG into interpreter flags

	rjmp	_dispatch_done_writeback_flags


exec_mov:
	movw	r6, r10
	movw	r8, r12
	rjmp	_dispatch_done_writeback_reg


exec_mul:
	clr	r21        ; Zero for adding carries
	clr	r22        ; Temporary for result
	clr	r23
	clr	r24
	clr	r25

	; Multiply, discarding all overflows because it's hilarious
	mul	r6, r10
	movw	r22, r0
	mul	r7, r11
	movw	r24, r0
	mul	r6, r11
	add	r23, r0
	adc	r24, r1
	adc	r25, r21
	mul	r7, r10
	add	r23, r0
	adc	r24, r1
	adc	r25, r21
	mul	r6, r12
	add	r24, r0
	adc	r25, r1
	mul	r8, r10
	add	r24, r0
	adc	r25, r1
	mul	r7, r12
	add	r25, r0
	mul	r8, r11
	add	r25, r0
	mul	r6, r13
	add	r25, r0
	mul	r9, r10
	add	r25, r0

	; Copy temporary to result
	movw	r6, r22
	movw	r8, r24

	; Don't compute any other flags, LOL!
	rjmp	_dispatch_done_writeback_flags


exec_div:
	; TODO
	rjmp	_dispatch_done_writeback_flags


exec_cmp:
	movw	r22, r6
	movw	r24, r8
	sub	r22, r10
	sbc	r23, r11
	sbc	r24, r12
	sbc	r25, r13
	in	r21, SREG
	andi	r21, 0x1d

	or	r25, r24
	or	r25, r23
	or	r25, r22
	brne	_cmp_nz
	ori	r21, 0x02
_cmp_nz:
	ldi	r25, 0xe0
	and	r14, r25
	or	r14, r21
	rjmp	_dispatch_done_writeback_fixedflags


exec_test:
	clr	r0
	mov	r25, r14
	andi	r25, 0xf9

	mov	r24, r6
	and	r24, r10
	or	r0, r24

	mov	r24, r7
	and	r24, r11
	or	r0, r24

	mov	r24, r8
	and	r24, r12
	or	r0, r24

	mov	r24, r9
	and	r24, r13
	sbrc	r24, 7
	ori	r25, 0x04
	or	r0, r24

	breq	_test_z
	ori	r25, 0x02
_test_z:
	mov	r14, r25
	rjmp	_dispatch_done_writeback_fixedflags


exec_jal_with_ve:
	; Change destination pointer to V[E]
	ldi	r24, 0x0e*4
	clr	r25
	ldi	r16, LOW(interp_regs)
	ldi	r17, HIGH(interp_regs)
	add	r16, r24
	adc	r17, r25
	; Fall-through to normal jump-and-link code
exec_jal:
	movw	r6, r2
	clr	r8
	clr	r9
	movw	r2, r10
	rjmp	_dispatch_done_writeback_reg


exec_shl:
	clr	r24        ; Zero for adding carries
	clr	r25        ; To accumulate carries

_shl_loop:
	dec	r10        ; Decrement counter
	brlt	_shl_done

	lsl	r6         ; Shift left by a bit
	rol	r7
	rol	r8
	rol	r9
	adc	r25, r24   ; Accumulate carries

	rjmp	_shl_loop
_shl_done:
	; TODO: CV flags
	rjmp	_dispatch_done_writeback_flags


exec_shrl:
	clr	r24        ; Zero for adding carries
	clr	r25        ; To accumulate carries

_shrl_loop:
	dec	r10        ; Decrement counter
	brlt	_shrl_done

	lsr	r9
	ror	r8
	ror	r7
	ror	r6
	adc	r25, r24   ; Accumulate carries

	rjmp	_shrl_loop
_shrl_done:
	; TODO: CV flags
	rjmp	_dispatch_done_writeback_flags


exec_shra:
	clr	r24        ; Zero for adding carries
	clr	r25        ; To accumulate carries

_shra_loop:
	dec	r10        ; Decrement counter
	brlt	_shra_done

	asr	r9
	ror	r8
	ror	r7
	ror	r6
	adc	r25, r24   ; Accumulate carries

	rjmp	_shra_loop
_shra_done:
	; TODO: CV flags
	rjmp	_dispatch_done_writeback_flags


exec_rol:
_rol_loop:
	dec	r10        ; Decrement counter
	brlt	_rol_done

	clc                ; Pull highest bit into carry
	sbrc	r9, 7
	sec
	rol	r6
	rol	r7
	rol	r8
	rol	r9

	rjmp	_rol_done
_rol_done:
	; TODO: Flags?
	rjmp	_dispatch_done_writeback_flags


exec_ror:
_ror_loop:
	dec	r10        ; Decrement counter
	brlt	_ror_done

	clc                ; Pull lowest bit into carry
	sbrc	r6, 0
	sec
	ror	r9
	ror	r8
	ror	r7
	ror	r6

	rjmp	_ror_done
_ror_done:
	; TODO: Flags?
	rjmp	_dispatch_done_writeback_flags


exec_spi:
	movw	ZL, r6
_spi_byte_loop:
	dec	r10
	brlt	_spi_done
	ld	r25, Z
	out	SPDR, r25
_spi_wait_loop:
	in	r25, SPSR
	sbrs	r25, SPIF
	rjmp	_spi_wait_loop
	in	r25, SPDR
	st	Z+, r25
	rjmp	_spi_byte_loop
_spi_done:
	rjmp	_dispatch_done


exec_mft:
	; TODO: countdown timer
	rjmp	_dispatch_done_writeback_reg


exec_mtt:
	; TODO: countdown timer
	rjmp	_dispatch_done


exec_din:
	; Read port values
	in	r24, PINB
	in	r25, PINC

	; Shift desired value into LSB
_din_loop:
	dec	r10
	brlt	_din_loop_done

	clc
	ror	r25
	ror	r24

	rjmp	_din_loop
_din_loop_done:

	; Extract port LSB and put into result LSB
	andi	r24, 0x01
	ldi	r25, 0xfe
	and	r6, r25
	or	r6, r24

	rjmp	_dispatch_done_writeback_flags


exec_dout:
	; Extract LSB from first operand
	ldi	r23, 0x01
	and	r6, r23
	clr	r7

	; Mask of all bits except LSB
	ldi	r24, 0xfe
	ldi	r25, 0xff

	; Rotate LSB and mask into position specified by second operand
_dout_loop:
	dec	r10
	brlt	_dout_loop_done

	sec
	rol	r24
	rol	r25

	clc
	rol	r6
	rol	r7

	rjmp	_dout_loop
_dout_loop_done:

	; Read-modify-write
	in	r22, PORTB
	in	r23, PORTC
	and	r22, r24
	and	r23, r25
	or	r22, r6
	or	r23, r7
	out	PORTB, r22
	out	PORTC, r23

	rjmp	_dispatch_done


exec_ain:
	; Set the ADC source
	lds	r25, ADMUX
	andi	r25, 0xf0
	mov	r24, r10
	andi	r24, 0x0f
	or	r25, r24
	sts	ADMUX, r25

	; Trigger a single conversion
	lds	r25, ADCSRA
	ori	r25, (1 << ADSC)
	sts	ADCSRA, r25

	; Wait for conversion to complete
_ain_wait:
	lds	r25, ADCSRA
	sbrs	r25, ADIF
	rjmp	_ain_wait
	sts	ADCSRA, r25

	; Read value from ADC
	lds	r6, ADCL
	lds	r7, ADCH
	clr	r8
	clr	r9

	rjmp	_dispatch_done_writeback_reg


exec_aout:
	; TODO: PWM output
	rjmp	_dispatch_done


exec_ldb:
	; Load byte
	movw	ZL, r10
	ld	r6, Z+

	; Sign extend
	clr	r0
	sbrc	r6, 7
	com	r0
	mov	r7, r0
	mov	r8, r0
	mov	r9, r0

	rjmp	_dispatch_done_writeback_reg


exec_ldh:
	; Load halfword
	movw	ZL, r10
	ld	r6, Z+
	ld	r7, Z+

	; Sign extend
	clr	r0
	sbrc	r7, 7
	com	r0
	mov	r8, r0
	mov	r9, r0

	rjmp	_dispatch_done_writeback_reg


exec_ldw:
	; Load word
	movw	ZL, r10
	ld	r6, Z+
	ld	r7, Z+
	ld	r8, Z+
	ld	r9, Z+

	rjmp	_dispatch_done_writeback_reg


exec_lpb:
	; Load byte
	movw	ZL, r10
	lpm	r6, Z+

	; Sign extend
	clr	r0
	sbrc	r6, 7
	com	r0
	mov	r7, r0
	mov	r8, r0
	mov	r9, r0

	rjmp	_dispatch_done_writeback_reg


exec_lph:
	; Load halfword
	movw	ZL, r10
	lpm	r6, Z+
	lpm	r7, Z+

	; Sign extend
	clr	r0
	sbrc	r7, 7
	com	r0
	mov	r8, r0
	mov	r9, r0

	rjmp	_dispatch_done_writeback_reg


exec_lpw:
	; Load word
	movw	ZL, r10
	lpm	r6, Z+
	lpm	r7, Z+
	lpm	r8, Z+
	lpm	r9, Z+

	rjmp	_dispatch_done_writeback_reg


exec_stb:
	movw	ZL, r10
	st	Z+, r6
	rjmp	_dispatch_done


exec_sth:
	movw	ZL, r10
	st	Z+, r6
	st	Z+, r7
	rjmp	_dispatch_done


exec_stw:
	movw	ZL, r10
	st	Z+, r6
	st	Z+, r7
	st	Z+, r8
	st	Z+, r9
	rjmp	_dispatch_done


exec_ext:
	; Can't use regular call/ret instructions, they take more than 3 cycles

	; Put the return address into temporaries
	ldi	r24, LOW(_ext_done)
	ldi	r25, HIGH(_ext_done)

	; Execute at the target address
	movw	ZL, r10
	ijmp
_ext_done:
	rjmp	_dispatch_done


exec_jtab:
	add	r10, r6    ; Add V[X] to PC+sext(nn)
	adc	r11, r7
	movw	r2, r10
	rjmp	_dispatch_done


exec_blt:
	sbrc	r14, 4
	movw	r2, r10    ; Branch if S bit is set
	rjmp	_dispatch_done
exec_bge:
	sbrs	r14, 4
	movw	r2, r10    ; Branch if S bit is clear
	rjmp	_dispatch_done


exec_bv:
	sbrc	r14, 3
	movw	r2, r10    ; Branch if V bit is set
	rjmp	_dispatch_done
exec_bnv:
	sbrs	r14, 3
	movw	r2, r10    ; Branch if V bit is clear
	rjmp	_dispatch_done


exec_bmi:
	sbrc	r14, 2
	movw	r2, r10    ; Branch if N bit is set
	rjmp	_dispatch_done
exec_bpl:
	sbrs	r14, 2
	movw	r2, r10    ; Branch if N bit is clear
	rjmp	_dispatch_done


exec_bz:
	sbrc	r14, 1
	movw	r2, r10    ; Branch if Z bit is set
	rjmp	_dispatch_done
exec_bnz:
	sbrs	r14, 1
	movw	r2, r10    ; Branch if Z bit is clear
	rjmp	_dispatch_done


exec_c:
	sbrc	r14, 0
	movw	r2, r10    ; Branch if C bit is set
	rjmp	_dispatch_done
exec_nc:
	sbrs	r14, 0
	movw	r2, r10    ; Branch if C bit is clear
	rjmp	_dispatch_done


exec_jmp:
	movw	r2, r10
	rjmp	_dispatch_done


; ------------------------------------------------------------------------------
; Interpreter state

.dseg

; 16 general purpose 32-bit "registers" (V0-VF)
interp_regs:
	.byte	16*4

