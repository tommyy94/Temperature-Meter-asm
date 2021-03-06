;******************************************************************************;
;   HEADER FILES
;******************************************************************************;
.nolist
.include "main.inc"
.list


;******************************************************************************;
;   RESET AND INT VECTORS
;******************************************************************************;
.cseg
.org $0000 ;interrupts located at start of the flash

jmp reset               ; Reset
reti nop                ; INT0
reti nop                ; INT1
jmp PCINT0_vect         ; PCINT0
reti nop                ; PCINT1
reti nop                ; PCINT2
reti nop                ; WDT
jmp TIM2_COMPA2_vect    ; TIM2_COMPA
reti nop                ; TIM2_COMPB
reti nop                ; TIM2_OVF
reti nop                ; TIM1_CAPT
reti nop                ; TIM1_COMPA
reti nop                ; TIM1_COMPB
reti nop                ; TIM1_OVF
reti nop                ; TIM0_COMPA
reti nop                ; TIM0_COMPB
reti nop                ; TIM0_OVF
reti nop                ; SPI_STC
reti nop                ; USART_RXC
reti nop                ; USART_UDRE
reti nop                ; USART_TXC
reti nop                ; ADC
reti nop                ; EE_RDY
reti nop                ; ANA_COMP
reti nop                ; TWI
reti nop                ; SPM_RDY


;******************************************************************************;
;   INITIALIZE
;******************************************************************************
.cseg
.org $0300

reset:
    INIT_STACK_POINTER
    SET_CLK_PRESCALER ;using 8 MHz clock speed

    INIT_TIMER2 ;CTC on 10 ms
    INIT_SLEEP ;using Power-save Mode

    INIT_ADC ;ADC on channel 0, 125 kHz resolution

    INIT_PCINT0_VECT ;on PORTB and SYS_RESET_PIN
    INIT_SYS_RESET_PIN

    rcall lcd_init
    

;******************************************************************************;
;    MAIN LOOP
;******************************************************************************;
main:
    rcall adc_read ;result stored in r19:r18
    CELSIUS_FORMULA
    ;FAHRENHEIT_FORMULA r19, r18
    ;KELVIN_FORMULA r19, r18

    rcall integer_to_ascii

    ;load parameters for subroutine
    ldi r21, FIRST_COLUMN
    ldi r22, FIRST_ROW_POS

	ldi ZH, HIGH(string_to_send) ;program memory is stored in 16-bit words, so
	ldi ZL, LOW(string_to_send)  ;multiply by 2 to extract bytes
    ldi r16, 4 ;counter

    rcall lcd_send_string
    
    rcall sleep_10ms
	rjmp main

    
;******************************************************************************;
;   INTERRUPT SERVICE ROUTINES
;******************************************************************************;

;******************************************************************************;
; PCINT0_vect
; Registers used: none
; Description: To be used for watchdog system reset.
;******************************************************************************;
PCINT0_vect:
    ;check if SYS_RESET_PIN is pressed
    sbic PINB, SYS_RESET_PIN ;falling edge
    nop
    reti


.exit
