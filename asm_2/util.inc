; Include guards are broken in AVRA, so these won't save you.
#ifndef _UTIL_INC_
#define _UTIL_INC_


; ------------------------------------------------------------------------------
; The assembler packaged with avr_sim has a bug where PC evaluates to PC-1.
; The avra assembler gets it correct, but I can't get the supposedly predefined
; __AVRASM_VERSION__ macro to work, so for now just manually enter the error in
; PC in this symbol.

; Use with AVRA assembler
; "AVRA: advanced AVR macro assembler Version 1.3.0 Build 1 (8 May 2010)"
.equ	PCERROR = 0

; Use with assembler packaged with avr_sim
; "gavrasm Gerd's AVR assembler version 5.4 (C)2022 by DG4FAC"
;.equ	PCERROR = -1


; ------------------------------------------------------------------------------
; The AVR assembler doesn't come with an alignment directive

.macro ALIGN
	.if ((PC-PCERROR) % @0) != 0
	.org (PC-PCERROR) + (@0 - ((PC-PCERROR) % @0))
	.endif
.endmacro


#endif  /* _UTIL_INC_ */

