.include "./m328Pdef.inc"


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
; r11  |___ Operand B / instruction result part 2 (mul/div)
; r12  |
; r13 h/
; r14 ----- New flags register value (VF)
; r15
; r16 l\___ Pointer to V[X]
; r17 h/
; r18
; r19
; r20
; r21
; r22
; r23
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
; Reset entry point

reset:
	; ----- Disable interrupts and set stack pointer to top of SRAM

	eor	r25, r25
	out	SREG, r25
	ldi	r25, HIGH(RAMEND)
	out	SPH, r25
	ldi	r25, LOW(RAMEND)
	out	SPL, r25

	; ----- Reset interpreter state

	; TODO

	; --------------------------------------------------------------
	; Main loop
loop:
	; Check for pin change interrupt and debounce
	; TODO

	; Fetch instruction
	movw	ZL, r2
	lpm	r4, Z+
	lpm	r5, Z+
	movw	r2, ZL

	; Decode first operand, always register V[X]
	mov	r16, r5
	andi	r16, 0x0f
	lsl	r16
	lsl	r16
	ldi	r17, 0x00
	ldi	ZL, LOW(interp_regs)
	ldi	ZH, HIGH(interp_regs)
	movw	r16, ZL
	ld	r6, Z+
	ld	r7, Z+
	ld	r8, Z+
	ld	r9, Z+

	; Decode second operand based on instruction type
	mov	r10, r5
	asr	r10
	asr	r10
	asr	r10
	asr	r10
	eor	r0, r0
	ldi	ZL, LOW(operand_jumptable)
	ldi	ZH, HIGH(operand_jumptable)
	add	ZL, r10
	adc	ZH, r0
	ijmp
_decode_done:

	; Dispatch based on instruction type
	ldi	ZL, LOW(f_dispatch_jumptable)
	ldi	ZH, HIGH(f_dispatch_jumptable)
	eor	r0, r0
	add	ZL, r10
	adc	ZH, r0
_dispatch_done_writeback_flags:
	ldi	ZL, LOW(interp_regs)
	ldi	ZH, HIGH(interp_regs)
	ldi	r25, 0x0e*4
	add	ZL, r25
	eor	r0, r0
	adc	ZH, r0
	st	Z, r14
_dispatch_done_writeback_reg:
	movw	ZL, r16
	st	Z+, r6
	st	Z+, r7
	st	Z+, r8
	st	Z+, r9
_dispatch_done:

	rjmp	loop


; ------------------------------------------------------------------------------
; Operand decoding

operand_jumptable:
	.dw	operand_VY
	.dw	operand_imm32
	.dw	operand_Y
	.dw	operand_VY_N
	.dw	operand_VY_N
	.dw	operand_VY_N
	.dw	operand_VY_N
	.dw	operand_VY_N
	.dw	operand_VY_N
	.dw	operand_none
	.dw	operand_0NNN
	.dw	operand_PC_ssNN
	.dw	operand_PC_sNNN
	.dw	operand_PC_sNNN
	.dw	operand_none
	.dw	operand_none

operand_VY:
	rjmp	_decode_done

operand_imm32:
	rjmp	_decode_done

operand_Y:
	rjmp	_decode_done

operand_VY_N:
	rjmp	_decode_done

operand_ssNN:
	rjmp	_decode_done

operand_0NNN:
	rjmp	_decode_done

operand_PC_ssNN:
	rjmp	_decode_done

operand_PC_sNNN:
	rjmp	_decode_done

operand_none:
	rjmp	_decode_done


; ------------------------------------------------------------------------------
; Instruction dispatch

f_dispatch_jumptable:
	.dw	dispatch_alu
	.dw	dispatch_alu
	.dw	dispatch_imm4
	.dw	exec_ldb
	.dw	exec_ldh
	.dw	exec_ldw
	.dw	exec_stb
	.dw	exec_sth
	.dw	exec_stw
	.dw	exec_nop
	.dw	exec_avr
	.dw	dispatch_branch
	.dw	exec_jal_with_ve
	.dw	exec_jmp
	.dw	exec_nop
	.dw	exec_nop

alu_dispatch_jumptable:
	.dw	exec_add
	.dw	exec_sub
	.dw	exec_and
	.dw	exec_or
	.dw	exec_xor
	.dw	exec_nor
	.dw	exec_mov
	.dw	exec_mul
	.dw	exec_div
	.dw	exec_cmp
	.dw	exec_test
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_jal

imm4_dispatch_jumptable:
	.dw	exec_add
	.dw	exec_sub
	.dw	exec_shl
	.dw	exec_shrl
	.dw	exec_shra
	.dw	exec_rol
	.dw	exec_ror
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_spi
	.dw	exec_mft
	.dw	exec_mtt
	.dw	exec_din
	.dw	exec_dout
	.dw	exec_ain
	.dw	exec_aout

branch_dispatch_jumptable:
	.dw	exec_bz
	.dw	exec_bnz
	.dw	exec_bgt
	.dw	exec_blt
	.dw	exec_blte
	.dw	exec_bgte
	.dw	exec_bc
	.dw	exec_bnc
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_nop
	.dw	exec_nop


dispatch_alu:
	ldi	ZL, LOW(alu_dispatch_jumptable)
	ldi	ZH, HIGH(alu_dispatch_jumptable)
	mov	r25, r4
	andi	r25, 0x0f
	eor	r1, r1
	add	ZL, r0
	adc	ZH, r1
	ijmp

dispatch_imm4:
	ldi	ZL, LOW(imm4_dispatch_jumptable)
	ldi	ZH, HIGH(imm4_dispatch_jumptable)
	mov	r25, r4
	andi	r25, 0x0f
	eor	r1, r1
	add	ZL, r0
	adc	ZH, r1
	ijmp

dispatch_branch:
	ldi	ZL, LOW(branch_dispatch_jumptable)
	ldi	ZH, HIGH(branch_dispatch_jumptable)
	mov	r25, r5
	andi	r25, 0x0f
	eor	r1, r1
	add	ZL, r0
	adc	ZH, r1
	ijmp

exec_nop:
exec_add:
exec_sub:
exec_and:
exec_or:
exec_xor:
exec_nor:
exec_mov:
exec_mul:
exec_div:
exec_cmp:
exec_test:
exec_jal:
exec_shl:
exec_shrl:
exec_shra:
exec_rol:
exec_ror:
exec_spi:
exec_mft:
exec_mtt:
exec_din:
exec_dout:
exec_ain:
exec_aout:
exec_ldb:
exec_ldh:
exec_ldw:
exec_stb:
exec_sth:
exec_stw:
exec_avr:
exec_bz:
exec_bnz:
exec_bgt:
exec_blt:
exec_blte:
exec_bgte:
exec_bc:
exec_bnc:
exec_jal_with_ve:
exec_jmp:
	rjmp	_dispatch_done


; ------------------------------------------------------------------------------
; Interpreter state

.dseg

; 16 general purpose 32-bit "registers" (V0-VF)
interp_regs:
	.byte	16*4

