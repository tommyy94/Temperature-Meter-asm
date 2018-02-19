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
;******************************************************************************;
reset: 
    INIT_STACK_POINTER

    SET_CLK_PRESCALER ;using 8 MHz clock speed

init_timer2: ;8-bit timer
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

    INIT_ADC

init_sleep:
    ;using Power-save Mode
    ldi tmp_reg0, (1 << SM1) | (1 << SM0)
    out SMCR, tmp_reg0

init_PCINT0_vect:
    ;set SYS_RESET_PIN as input and enable it
    in tmp_reg0, DDRB
    andi tmp_reg0, ~(1 << SYS_RESET_PIN)
    out DDRB, tmp_reg0
    ldi tmp_reg0, (1 << SYS_RESET_PIN)
    out PORTB, tmp_reg0

init_interrupts:
    ;change on PCINT[7:0] fires an interrupt
    ldi tmp_reg0, (1 << PCIE0)
    sts PCICR, tmp_reg0
    ;enable PCINT[7:0] pins
    ldi tmp_reg0, (1 << PCINT4)
    sts PCMSK0, tmp_reg0

    rcall lcd_init
    

;******************************************************************************;
;    MAIN LOOP
;******************************************************************************;
main:
    rcall adc_read
    rcall lcd_send_character
    
    rcall sleep_10ms

	rjmp main

    
;******************************************************************************;
; SUBROUTINE: delay_10_us
; Registers used: tmp_reg0
; Description: 10 microsecond delay loop.
;******************************************************************************;
;
; T = 1/f = 1 / 8 * 10^6 Hz = 0.1250 us/cycle
;
; total_cycles = delay / T
;              = 10 us * 0.1250 us
;              = 80 cycles
;
; finding value for counter:
; counter = total_cycles - rcall - ret - ldi = 72 cycles
; 
; when brne = true, 2 cycles + 1 cycle from dec = 3 cycles
; => counter = 72 / 3
;            = 24
; last branch has to be false (1 cycle), therefore minus 1 cycle
; => counter = 24-1
;            = 23
; 
; checking everything adds up:
; rcall + ldi + dec + (brne true + dec) * counter + (brne false) + ret = 79
; => 79 + nop = 80 cycles = 10 us
;
delay_10_us:                                ;3 cycles
    ldi tmp_reg0, COUNTER_DELAY_10_US       ;1 cycle
delay_10_us_loop:
    dec tmp_reg0                            ;1 cycle
    brne delay_10_us_loop                   ;3 * 23 + 1
    nop                                     ;1 cycle
    ret                                     ;4 cycles

    
;******************************************************************************;
; SUBROUTINE: sleep_10ms
; Registers used: tmp_reg0
; Description: Sleeps for 10 ms. Writes Sleep Enable bit to logic one and sleep 
; until interrupt occurs, then clears Sleep Enable bit.
;******************************************************************************;
sleep_10ms:
    push tmp_reg0
    
    sei ;enable interrupts so we can wake up

    ;enable sleep
    in tmp_reg0, SMCR
    ori tmp_reg0, (1 << SE)
    out SMCR, tmp_reg0

    sleep

    ;disable sleep
    andi tmp_reg0, ~(1 << SE)
    out SMCR, tmp_reg0
    
    pop tmp_reg0
    ret

    
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
    sbic PINB, SYS_RESET_PIN       ;falling edge
    nop ;debug message: PCINT0
    reti

    
;******************************************************************************;
; TIM2_COMPA2_vect
; Registers used: none
; Description: Used to wake up the processor.
;******************************************************************************;
TIM2_COMPA2_vect:
    nop ;used to wake up from sleep
    reti


.exit
