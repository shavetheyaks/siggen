; ------------------------------------------------------------------------------
; Tortoise Bytecode Assembly Macros and Quick Reference
; ------------------------------------------------------------------------------
;
; Registers:
;
; V0 / TM -- Temporary
; V1 / A0 \
; V2 / A1 |
; V3 / A2 |_ Argument/Caller-saved
; V4 / A3 |
; V5 / A4 |
; V6 / A5 /
; V7 / S0 \
; V8 / S1 |
; V9 / S2 |_ Callee-saved
; VA / S3 |
; VB / S4 |
; VC / S5 /
; VD / SP -- Stack Pointer
; VE / RA -- Return Address
; VF / FL -- Flags (---svnzc)
;
;
; Instructions and pseudoinstructions:
;
; T_ADD   vx, vy     Add                T_ADDI   vx, imm32   Add
; T_SUB   vx, vy     Subtract           T_SUBI   vx, imm32   Subtract
; T_AND   vx, vy     Logical AND        T_ANDI   vx, imm32   Logical AND
; T_OR    vx, vy     Logical OR         T_ORI    vx, imm32   Logical OR
; T_XOR   vx, vy     Logical XOR        T_XORI   vx, imm32   Logical XOR
; T_NOR   vx, vy     Logical NOR        T_NORI   vx, imm32   Logical NOR
; T_MOV   vx, vy     Copy value         T_MOVI   vx, imm32   Copy value
; T_MUL   vx, vy     Multiply           T_MULI   vx, imm32   Multiply
; T_DIV   vx, vy     Divide             T_DIVI   vx, imm32   Divide
; T_CMP   vx, vy     Compare            T_CMPI   vx, imm32   Compare
; T_TEST  vx, vy     Test (AND)         T_TESTI  vx, imm32   Test (AND)
; T_NOT   vx         Logical NOT        T_NEG    vx          2's complement
; T_CLR   vx         Load zero          T_NOP                Do nothing
; T_ADDS  vx, imm4   Add                T_SHRL   vx, imm4    Log. shift right
; T_SUBS  vx, imm4   Subtract           T_SHRA   vx, imm4    Arith. shift right
; T_MOVS  vx, imm4   Copy value         T_ROL    vx, imm4    Rotate left
; T_SHL   vx, imm4   Shift left         T_ROR    vx, imm4    Rotate right
;
;
; T_LDB   vx, vy, n  Load 8  [VY+n]     T_STB    vx, vy, n   Store 8  [VY+n]
; T_LDH   vx, vy, n  Load 16 [VY+n]     T_STH    vx, vy, n   Store 16 [VY+n]
; T_LDW   vx, vy, n  Load 32 [VY+n]     T_STW    vx, vy, n   Store 32 [VY+n]
; T_LPB   vx, vy, n  Load ROM 8  [VY+n]
; T_LPH   vx, vy, n  Load ROM 16 [VY+n]
; T_LPW   vx, vy, n  Load ROM 32 [VY+n]
;
;
; T_BZ    label      Branch if zero
; T_BNZ   label      Branch if not zero
; T_BEQ   label      Branch if equal
; T_BNE   label      Branch if not equal
; T_BLT   label      Branch if less-than (signed)
; T_BGE   label      Branch if greater-than or equal (signed)
; T_BLO   label      Branch if lower (unsigned)
; T_BAE   label      Branch if above or equal (unsigned)
; T_BMI   label      Branch if minus (negative)
; T_BPL   label      Branch if plus (zero or positive)
; T_BC    label      Branch if carry
; T_BNC   label      Branch if no carry
; T_BV    label      Branch if overflow
; T_BNV   label      Branch if no overflow
;
;
; T_JMP   label      Jump to label
; T_JTAB  vx, label  Jump to label+VX       (V0-V5 only)
; T_JAL   label      Jump to label,         return address in VE/RA
; T_JALR  vx, vy     Jump to address in VY, return address in VX
; T_JALI  vx, imm32  Jump to imm32,         return address in VX
; T_EXT   label      Jump to label and execute as AVR assembly
;                    (return address in r24:r25)
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
; Variable/register definitions

.equ	V0 = 0x0
.equ	V1 = 0x1
.equ	V2 = 0x2
.equ	V3 = 0x3
.equ	V4 = 0x4
.equ	V5 = 0x5
.equ	V6 = 0x6
.equ	V7 = 0x7
.equ	V8 = 0x8
.equ	V9 = 0x9
.equ	VA = 0xa
.equ	VB = 0xb
.equ	VC = 0xc
.equ	VD = 0xd
.equ	VE = 0xe
.equ	VF = 0xf

; Alternate names
.equ	TM = V0
.equ	A0 = V1
.equ	A1 = V2
.equ	A2 = V3
.equ	A3 = V4
.equ	A4 = V5
.equ	A5 = V6
.equ	S0 = V7
.equ	S1 = V8
.equ	S2 = V9
.equ	S3 = VA
.equ	S4 = VB
.equ	S5 = VC
.equ	SP = VD
.equ	RA = VE
.equ	FL = Vf


; ------------------------------------------------------------------------------
; Instruction building macros


; ----- Group 0x0, register-register

.macro T_ADD
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x0)
.endmacro

.macro T_SUB
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x1)
.endmacro

.macro T_AND
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x2)
.endmacro

.macro T_OR
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x3)
.endmacro

.macro T_XOR
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x4)
.endmacro

.macro T_NOR
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x5)
.endmacro

.macro T_MOV
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x6)
.endmacro

.macro T_MUL
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x7)
.endmacro

.macro T_DIV
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x8)
.endmacro

.macro T_CMP
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0x9)
.endmacro

.macro T_TEST
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0xa)
.endmacro

.macro T_JALR
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x0 << 12) | (@0 << 8) | (@1 << 4) | (0xf)
.endmacro

.macro T_NOT
	T_NOR	@0, @0
.endmacro


; ----- Group 0x1, register-immediate (32-bit)

.macro T_ADDI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x0)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_SUBI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x1)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_ANDI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x2)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_ORI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x3)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_XORI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x4)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_NORI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x5)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_MOVI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x6)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_MULI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x7)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_DIVI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x8)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_CMPI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0x9)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_TESTI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0xa)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro

.macro T_JALI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.dw	(0x1 << 12) | (@0 << 8) | (0 << 4) | (0xf)
	.dw	LWRD(@1)
	.dw	HWRD(@1)
.endmacro


; ----- Group 2, register-immediate (4-bit)

.macro T_ADDS
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0x0)
.endmacro

.macro T_SUBS
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0x1)
.endmacro

.macro T_MOVS
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0x2)
.endmacro

.macro T_SHL
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0x3)
.endmacro

.macro T_SHRL
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0x4)
.endmacro

.macro T_SHRA
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0x5)
.endmacro

.macro T_ROL
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0x6)
.endmacro

.macro T_ROR
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0x7)
.endmacro

.macro T_SPI
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0x9)
.endmacro

.macro T_MFT
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0xa)
.endmacro

.macro T_MTT
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0xb)
.endmacro

.macro T_DIN
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0xc)
.endmacro

.macro T_DOUT
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0xd)
.endmacro

.macro T_AIN
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0xe)
.endmacro

.macro T_AOUT
	.if @0 < 0 || @0 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x2 << 12) | (@0 << 8) | (@1 << 4) | (0xf)
.endmacro

.macro T_NEG
	T_NOT	@0
	T_ADDS	@0, 1
.endmacro


; ----- Groups 3-A,F, Load/store

.macro T_LDB
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @2 < 0 || @2 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x3 << 12) | (@0 << 8) | (@1 << 4) | (@2)
.endmacro

.macro T_LDH
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @2 < 0 || @2 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x4 << 12) | (@0 << 8) | (@1 << 4) | (@2)
.endmacro

.macro T_LDW
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @2 < 0 || @2 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x5 << 12) | (@0 << 8) | (@1 << 4) | (@2)
.endmacro

.macro T_STB
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @2 < 0 || @2 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x6 << 12) | (@0 << 8) | (@1 << 4) | (@2)
.endmacro

.macro T_STH
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @2 < 0 || @2 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x7 << 12) | (@0 << 8) | (@1 << 4) | (@2)
.endmacro

.macro T_STW
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @2 < 0 || @2 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x8 << 12) | (@0 << 8) | (@1 << 4) | (@2)
.endmacro

.macro T_LPB
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @2 < 0 || @2 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0x9 << 12) | (@0 << 8) | (@1 << 4) | (@2)
.endmacro

.macro T_LPH
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @2 < 0 || @2 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0xa << 12) | (@0 << 8) | (@1 << 4) | (@2)
.endmacro

.macro T_LPW
	.if @0 < 0 || @0 > 0xf || @1 < 0 || @1 > 0xf
	.error "Tortoise Bytecode: invalid variable register"
	.endif
	.if @2 < 0 || @2 > 0xf
	.error "Tortoise Bytecode: immediate value outside 4-bit range"
	.endif
	.dw	(0xf << 12) | (@0 << 8) | (@1 << 4) | (@2)
.endmacro


; ----- Group B, branches

.macro T_JTAB
	.if @0 < 0 || @0 > 0x5
	.error "Tortoise Bytecode: invalid variable register (V0-V5 for JTAB)"
	.endif
	.if (@1-(PC+1)) < -0x80 || (@1-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xb << 12) | (@0 << 8) | (@1-(PC+1))
.endmacro

.macro T_BLT
	.if (@0-(PC+1)) < -0x80 || (@0-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xb6 << 8) | (@0-(PC+1))
.endmacro

.macro T_BGE
	.if (@0-(PC+1)) < -0x80 || (@0-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xb7 << 8) | (@0-(PC+1))
.endmacro

.macro T_BV
	.if (@0-(PC+1)) < -0x80 || (@0-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xb8 << 8) | (@0-(PC+1))
.endmacro

.macro T_BNV
	.if (@0-(PC+1)) < -0x80 || (@0-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xb9 << 8) | (@0-(PC+1))
.endmacro

.macro T_BMI
	.if (@0-(PC+1)) < -0x80 || (@0-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xba << 8) | (@0-(PC+1))
.endmacro

.macro T_BPL
	.if (@0-(PC+1)) < -0x80 || (@0-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xbb << 8) | (@0-(PC+1))
.endmacro

.macro T_BZ
	.if (@0-(PC+1)) < -0x80 || (@0-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xbc << 8) | (@0-(PC+1))
.endmacro

.macro T_BNZ
	.if (@0-(PC+1)) < -0x80 || (@0(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xbd << 8) | (@0-(PC+1))
.endmacro

.macro T_BC
	.if (@0-(PC+1)) < -0x80 || (@0-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xbe << 8) | (@0-(PC+1))
.endmacro

.macro T_BNC
	.if (@0-(PC+1)) < -0x80 || (@0-(PC+1)) >= 0x80
	.error "Tortoise Bytecode: branch target out of range"
	.endif
	.dw	(0xbf << 8) | (@0-(PC+1))
.endmacro

.macro T_BEQ
	T_BZ	@0
.endmacro

.macro T_BNE
	T_BNZ	@0
.endmacro

.macro T_BLO
	T_BC	@0
.endmacro

.macro T_BAE
	T_BNC	@0
.endmacro


; ----- Miscellaneous

.macro T_JAL
	.if (@0-(PC+1)) < -0x800 || (@0-(PC+1)) >= 0x800
	.error "Tortoise Bytecode: jump target out of range"
	.endif
	.dw	(0xc << 12) | ((HIGH(@0-(PC+1))&0x0f) << 8) | LOW(@0-(PC+1))
.endmacro

.macro T_JMP
	.if (@0-(PC+1)) < -0x800 || (@0-(PC+1)) >= 0x800
	.error "Tortoise Bytecode: jump target out of range"
	.endif
	.dw	(0xd << 12) | ((HIGH(@0-(PC+1))&0x0f) << 8) | LOW(@0-(PC+1))
.endmacro

.macro T_EXT
	.if @0 < 0 || @0 > 0xfff
	.error "Tortoise Bytecode: EXT target out of range"
	.endif
	.dw	(0xe << 12) | (HIGH(@0)&0x0f) | LOW(@0)
.endmacro

.macro T_CLR
	T_XOR	@0, @0
.endmacro

.macro T_NOP
	.dw	0x000E
.endmacro

