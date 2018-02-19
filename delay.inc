;******************************************************************************;
;   HEADER FILES
;******************************************************************************;
.nolist
.include "delay.asm"
.list

;******************************************************************************;
;   HARDWARE INFORMATION
;******************************************************************************;
;8 MHz system clock speed


;******************************************************************************;
;   PORTS AND PINS
;******************************************************************************;


;******************************************************************************;
;   CONSTANTS
;******************************************************************************;


;******************************************************************************;
;   FIXED + DERIVED CONSTANTS
;******************************************************************************;
.equ COUNTER_DELAY_10_US = 24


;******************************************************************************;
;   REGISTER DEFINITIONS
;******************************************************************************;


;******************************************************************************;
;   SRAM DEFINITIONS
;******************************************************************************;


;******************************************************************************;
;   MACROS
;******************************************************************************;
.listmac ;list macros in .lst file

;******************************************************************************;
; MACRO: INIT_TIMER2
; Registers used: tmp_reg0
; Description: Initializes timer2 (8-bit), 10 ms CTC mode.
;******************************************************************************
.macro INIT_TIMER2
    ; timer_ticks = CPU frequency / timer prescaler
    ;             = 8 MHz / 1024 clk
    ;             = 7812.5 timer ticks (=> 1000 ms)
    ;
    ; converting to milliseconds:
    ; duration_as_hex = timer_ticks * <time to sleep in milliseconds>
    ; NOTE: $00 is one count!
    
    ;CTC mode
    ldi tmp_reg0, (1 << WGM21)
    sts TCCR2A, tmp_reg0

    ;set timer prescaler to 1024
    ldi tmp_reg0, (1 << CS22) | (1 << CS21) | (1 << CS20)
    sts TCCR2B, tmp_reg0

    ;7812.5 ticks / (1000ms / target_delay) = target_delay_in_ticks
    ;                                       => convert to hexadecimal - 1,
    ;                                          because 0x00 is one count
    ;7812.5 ticks / (1000ms / 10ms) = 78 ticks
    ;                               => $4E -1
    ;                               = $4D
    ldi tmp_reg0, $4D
    sts OCR2A, tmp_reg0

    /* using TIMER2_COMPA interrupt */
    ldi tmp_reg0, (1 << OCIE2A)
    sts TIMSK2, tmp_reg0
.endmacro


;******************************************************************************;
; MACRO: INIT_SLEEP
; Registers used: tmp_reg0
; Description: Initializes sleep mode to user power-save mode.
;******************************************************************************
.macro INIT_SLEEP
    ldi tmp_reg0, (1 << SM1) | (1 << SM0)
    out SMCR, tmp_reg0
.endmacro


.exit
