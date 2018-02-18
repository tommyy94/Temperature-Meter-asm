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
.equ GREEN_LED = PINB6
.equ YELLOW_LED = PINB7
.equ SYS_RESET_PIN = PINB4


; ============================================
;   CONSTANTS
; ============================================

;==============================================================================;
; - EN (enable) control line informs the LCD, that data is being sent.
;   Set to HIGH, wait required time, then set LOW to execute instruction.
; - RS (register select), set LOW to treat data as command/special
;   instruction, HIGH treats data as text.
; - RW (read/write), LOW to write on LCD, HIGH to read LCD.
;==============================================================================;

;LCD CONSTANTS
.equ EN = PINB0
.equ RW = PINB3
.equ RS = PINB2
.equ DATA_LINES = PORTD
.equ DATA_DIRECTION = DDRD
.equ CONTROL_LINES = PORTB
.equ CONTROL_DIRECTION = DDRB

.equ FIRST_LINE_ADDRESS = 0x80
.equ SECOND_LINE_ADDRESS = 0xC0
.equ THIRD_LINE_ADDRESS = 0x94
.equ FOURTH_LINE_ADDRESS = 0xD4

.equ CLEAR_DISPLAY = 0x01
.equ SET_4_BIT = 0x28
.equ SET_8_BIT = 0x38
.equ DISPLAY_ON_CURSOR_OFF = 0x0C
.equ DISPLAY_ON_CURSOR_ON = 0x0E
.equ DISPLAY_ON_CURSOR_BLINK = 0x0F


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
.def param_reg0 = r19
.def param_reg1 = r20
.def pointer_reg_low = r24
.def pointer_reg_high = r25


; ============================================
;   SRAM DEFINITIONS
; ============================================


; ============================================
;   MACROS
; ============================================
.listmac ;list macros in .lst file

.macro INIT_STACK_POINTER
    ldi tmp_reg0, HIGH(RAMEND)
    out SPH, tmp_reg0
    ldi tmp_reg0, LOW(RAMEND)
    out SPL,  tmp_reg0
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

    sbic @1, @0	                ;if bit clear, skip next line
    cbi @2, @0                  ;set bit low

    sbis @1, @0	                ;if bit high, skip next line
    sbi @2, @0                  ;set bit high
.endmacro


;used as interrupt prologue
.macro ISR_PUSH_TMP_REGS
    ;store temporary register to stack
    push tmp_reg2
    push tmp_reg1
    push tmp_reg0

    in tmp_reg0, SREG            ;read status register
    push tmp_reg0                ;save status register to stack
.endmacro


;used as interrupt epilogue
.macro ISR_POP_TMP_REGS
    pop tmp_reg0                  ;get previous flag register from stack
    out SREG, tmp_reg0            ;restore status register
    
    ;restore temporary registers from stack
    pop tmp_reg0
    pop tmp_reg1
    pop tmp_reg2
.endmacro


.macro SET_CLK_PRESCALER
    ;sets Clock Division Factor to 1
    ldi tmp_reg0, (1 << CLKPCE) ;set enable bit
    sts CLKPR, tmp_reg0
    andi tmp_reg0, ~$FF
    sts CLKPR, tmp_reg0 ;set division factor
.endmacro


.exit
