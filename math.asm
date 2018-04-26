;***************************************************************************
;
; "mpy16u" - 16x16 Bit Unsigned Multiplication
;
; This subroutine multiplies the two 16-bit register variables 
; r19:r18 and r17:r16.
; The result is placed in r21:r20:r19:r18.
;
; Low registers used: None
; High registers used: 7 (r18, r19, r16/r18, r17/r19, r20, r21, r22)	
;
; r16		;multiplicand low byte
; r17		;multiplicand high byte
; r18		;multiplier low byte
; r19		;multiplier high byte
; r18		;result byte 0 (LSB)
; r19		;result byte 1
; r20		;result byte 2
; r21		;result byte 3 (MSB)
; r22		;loop counter
;***************************************************************************
mpy16u:
	clr	r21     ;clear 2 highest bytes of result
	clr	r20
	ldi	r22, 16 ;init loop counter
	lsr	r19
	ror	r18

m16u_1:
	brcc noad8		;if bit 0 of multiplier set
	add	r20, r16	;add multiplicand Low to byte 2 of res
	adc	r21, r17	;add multiplicand high to byte 3 of res

noad8:
	ror	r21		;shift right result byte 3
	ror	r20		;rotate right result byte 2
	ror	r19		;rotate result byte 1 and multiplier High
	ror	r18		;rotate result byte 0 and multiplier Low
	dec	r22		;decrement loop counter
	brne m16u_1	;if not done, loop more
	ret

    
;******************************************************************************;
; SUBROUTINE: div16u
; Description: Divides the two 16-bit unsigned numbers
; "r17:r16" (dividend), "r19:r18" (divisor). 
; Result: "r17:r16"
; Remainder: "r15:r14"
; High registers: 5 (r16, r17, r18, r19, r20)
; Low registers: 2 (r14, r15)
;******************************************************************************;
div16u:
	clr	r14 ;clear remainder low byte
	sub	r15, r15 ;clear remainder high byte and carry
	ldi	r20, 17 ;init loop counter

div16u_1:
    ;shift left dividend
	rol	r16
	rol	r17
	dec	r20 ;counter--
    ;return if done
	brne div16u_2
	ret

div16u_2:
    ;shift dividend into remainder
	rol	r14
	rol	r15
	sub	r14, r18 ;remainder -= divisor
	sbc	r15, r19
    ;restore remainder if result negative
	brcc div16u_3
	add	r14, r18
	adc	r15, r19
	clc ;clear carry to be shifted into result
	rjmp div16u_1 ;else

div16u_3:
	sec ;set carry to be shifted into result
	rjmp div16u_1

    
;******************************************************************************;
;   INTERRUPT SERVICE ROUTINES
;******************************************************************************;


.exit
