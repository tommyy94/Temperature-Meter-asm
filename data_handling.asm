;******************************************************************************;
;   HEADER FILES
;******************************************************************************;


;******************************************************************************;
; SUBROUTINE: integer_to_ascii
; Registers used:
; Description: Converts a digit to ascii by ORing target with $30. Converted
; digit is stored in memory location pointed by YH:YL.
;******************************************************************************;
integer_to_ascii:
    ;load pointer to buffer
    ldi YH, HIGH(string_to_send)
    ldi YL, LOW(string_to_send)

    push r16 ;push unmodified result for later use

    ;find the first digit
    ldi r19, HIGH(10)
    ldi r18, LOW(10)
    rcall div16u

    push r16 ;push modified result for later use

    ;store the result digit into the buffer at $0100
    ori r16, $30    ;convert digit to ascii
    st Y+, r16      ;result always fits 8-bit reg

    ;find the second digit
    ldi r18, 10     ;multiplier
    pop r17         ;pop modified result
    pop r16         ;pop unmodified result
    mul r17, r18    ;mul instruction stores
    sub r16, r0     ;the result in r1:r0, in our case it's 8-bit

    ;store the result digit into the buffer at $0100
    ori r16, $30    ;convert digit to ascii
    st Y+, r16      ;result always fits 8-bit reg

    ;store degree and celsius sign
    ldi r16, DEGREE_SIGN
    st Y+, r16
    ldi r16, 'C'
    st Y+, r16

    ret

    
;******************************************************************************;
;   INTERRUPT SERVICE ROUTINES
;******************************************************************************;


.exit
