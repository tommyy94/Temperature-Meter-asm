;******************************************************************************;
;   HEADER FILES
;******************************************************************************;
.nolist
.include "m328pdef.inc"
.include "lcd.inc"
.include "adc.inc"
.include "delay.inc"
.include "data_handling.inc"
.include "math.inc"
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


;******************************************************************************;
;   REGISTER DEFINITIONS
;******************************************************************************;
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
; Registers used: r16
; Description: Initializes the stack pointer.
;******************************************************************************
.macro INIT_STACK_POINTER
    ldi r16, HIGH(RAMEND)
    out SPH, r16
    ldi r16, LOW(RAMEND)
    out SPL, r16
.endmacro


;******************************************************************************;
; MACRO: ISR_PUSH_TMP_REGS
; Registers used: r16
; Description: Used as interrupt prologue. Pushes temporary registers and status
; register to stack in case interrupt needs to use them.
; NOTES: Always pair with macro ISR_POP_TMP_REGS.
;******************************************************************************
.macro ISR_PUSH_TMP_REGS
    ;store temporary register to stack
    push r18
    push r17
    push r16

    in r16, SREG            ;read status register
    push r16                ;save status register to stack
.endmacro


;******************************************************************************;
; MACRO: ISR_POP_TMP_REGS
; Registers used: r16
; Description: Used as interrupt epilogue. Pops temporary registers and status
; register from the stack.
; NOTES: Always pair with macro ISR_PUSH_TMP_REGS.
;******************************************************************************
.macro ISR_POP_TMP_REGS
    pop r16                  ;get previous flag register from stack
    out SREG, r16            ;restore status register
    
    ;restore temporary registers from stack
    pop r16
    pop r17
    pop r18
.endmacro


;******************************************************************************;
; MACRO: SET_CLK_PRESCALER
; Registers used: r16
; Description: Sets internal system clock to 8 MHz.
;******************************************************************************
.macro SET_CLK_PRESCALER
    ;sets Clock Division Factor to 1
    ldi r16, (1 << CLKPCE) ;set enable bit, cleared after 4 clock cycles
    sts CLKPR, r16
    andi r16, ~$FF
    sts CLKPR, r16 ;set division factor
.endmacro


;******************************************************************************;
; MACRO: INIT_SYS_RESET_PIN
; Registers used: r16
; Description: Set SYS_RESET_PIN as input and enable it.
;******************************************************************************
.macro INIT_SYS_RESET_PIN
    in r16, DDRB
    andi r16, ~(1 << SYS_RESET_PIN)
    out DDRB, r16
    ldi r16, (1 << SYS_RESET_PIN)
    out PORTB, r16
.endmacro


;******************************************************************************;
; MACRO: INIT_PCINT0_VECT
; Registers used: r16
; Description: Set SYS_RESET_PIN as input and enable it.
;******************************************************************************
.macro INIT_PCINT0_VECT
    ;enable interrupts on PCINT[7:0], in this case it's PORTB
    ldi r16, (1 << PCIE0)
    sts PCICR, r16
    ;enable interrupt on PCINT4, in this case it's SYS_RESET_PIN/PINB4
    ldi r16, (1 << PCINT4)
    sts PCMSK0, r16
.endmacro


.exit
