;******************************************************************************;
;   HEADER FILES
;******************************************************************************;
.nolist
.include "m328pdef.inc"
.include "lcd.inc"
.include "adc.inc"
.list


;******************************************************************************;
;   HARDWARE INFORMATION
;******************************************************************************;
;ATmega328P microcontroller


;******************************************************************************;
;   PORTS AND PINS
;******************************************************************************;
.equ SYS_RESET_PIN = PINB4


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
.def tmp_reg0 = r16
.def tmp_reg1 = r17
.def tmp_reg2 = r18
.def param_reg0 = r19
.def param_reg1 = r20
.def pointer_reg_low = r24
.def pointer_reg_high = r25


;******************************************************************************;
;   SRAM DEFINITIONS
;******************************************************************************;


;******************************************************************************;
;   MACROS
;******************************************************************************;
.listmac ;list macros in .lst file

;******************************************************************************;
; MACRO: INIT_STACK_POINTER
; Registers used: tmp_reg0
; Description: Initializes the stack pointer.
;******************************************************************************
.macro INIT_STACK_POINTER
    ldi tmp_reg0, HIGH(RAMEND)
    out SPH, tmp_reg0
    ldi tmp_reg0, LOW(RAMEND)
    out SPL,  tmp_reg0
.endmacro

    
;******************************************************************************;
; MACRO: ISR_PUSH_TMP_REGS
; Registers used: tmp_reg0
; Description: Used as interrupt prologue. Pushes temporary registers and status
; register to stack in case interrupt needs to use them.
;******************************************************************************
.macro ISR_PUSH_TMP_REGS
    ;store temporary register to stack
    push tmp_reg2
    push tmp_reg1
    push tmp_reg0

    in tmp_reg0, SREG            ;read status register
    push tmp_reg0                ;save status register to stack
.endmacro


;******************************************************************************;
; MACRO: ISR_POP_TMP_REGS
; Registers used: tmp_reg0
; Description: Used as interrupt epilogue. Pops temporary registers and status
; register from the stack.
;******************************************************************************
;used as interrupt epilogue
.macro ISR_POP_TMP_REGS
    pop tmp_reg0                  ;get previous flag register from stack
    out SREG, tmp_reg0            ;restore status register
    
    ;restore temporary registers from stack
    pop tmp_reg0
    pop tmp_reg1
    pop tmp_reg2
.endmacro


;******************************************************************************;
; MACRO: SET_CLK_PRESCALER
; Registers used: tmp_reg0
; Description: Sets internal system clock to 8 MHz.
;******************************************************************************
.macro SET_CLK_PRESCALER
    ;sets Clock Division Factor to 1
    ldi tmp_reg0, (1 << CLKPCE) ;set enable bit, cleared after 4 clock cycles
    sts CLKPR, tmp_reg0
    andi tmp_reg0, ~$FF
    sts CLKPR, tmp_reg0 ;set division factor
.endmacro


.exit
