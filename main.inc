.nolist
.include "m328pdef.inc"
.list


; ============================================
;   HARDWARE INFORMATION
; ============================================
; F_CPU = 8 MHz
.equ F_CPU = 8000000

; ============================================
;   PORTS AND PINS
; ============================================
.equ THRESHOLD_LED = PINB3
.equ DEBUG_LED = PINB6
.equ SYS_RESET_PIN = PINB4

; ============================================
;   CONSTANTS
; ============================================
.equ ADC_THRESHOLD = $94 ;used for testing whether ADC works

; ============================================
;   FIXED + DERIVED CONSTANTS
; ============================================
.equ COUNTER_DELAY_10_US = 24

; ============================================
;   REGISTER DEFINITIONS
; ============================================
.def tmp_reg0 = r16
.def tmp_reg1 = r17
.def tmp_reg2 = r18
.def adc_reg_low = r19
.def adc_reg_high = r20
.def counter_reg0 = r24
.def counter_reg1 = r25


; ============================================
;   SRAM DEFINITIONS
; ============================================


; ============================================
;   MACROS
; ============================================
.listmac ;list macros in .lst file

.macro INIT_STACK_POINTER
    ldi   tmp_reg0, HIGH(RAMEND)
    out   SPH, tmp_reg0
    ldi   tmp_reg0, LOW(RAMEND)
    out   SPL,  tmp_reg0
.endmacro


;toggles selected bit, mainly for debugging purposes
.macro TOGGLE_BIT ;@bit, @pin, @port
    ;error checking
    .if (@0 < 0 || @0 > 7)
        .error "Invalid bit (valid bits are 0-7"
    .elif !(@1 == PINB || (@1 == PINC) || (@1 == PIND))
        .error "Invalid pin (valid pins: PINB, PINC, PIND)"
    .elif !(@2 == PORTB || (@2 == PORTC) || (@2 == PORTD))
        .error "Invalid port (valid ports: PORTB, PORTC, PORTD)"
    .endif

    sbic  @1, @0	                ;if bit clear, skip next line
    cbi   @2, @0                    ;set bit low

    sbis  @1, @0	                ;if bit high, skip next line
    sbi   @2, @0                    ;set bit high
.endmacro


;used as function prologue
.macro PUSH_TMP_REGS
    push  tmp_reg2
    push  tmp_reg1
    push  tmp_reg0

    in    tmp_reg0, SREG            ;read status register
    push  tmp_reg0                  ;save status register to stack
.endmacro


;used as function epilogue
.macro POP_TMP_REGS
    pop   tmp_reg0                  ;get previous flag register from stack
    out   SREG, tmp_reg0            ;restore status register
    
    ;restore temporary registers from stack
    pop   tmp_reg0
    pop   tmp_reg1
    pop   tmp_reg2
.endmacro
