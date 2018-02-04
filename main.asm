.nolist
.include "main.inc"
.list


; ============================================
;   RESET AND INT VECTORS
; ============================================
.cseg
.org $0000
jmp reset               ; Reset
reti nop                ; INT0
reti nop                ; INT1
jmp PCINT0_vect         ; PCINT0
reti nop                ; PCINT1
reti nop                ; PCINT2
reti nop                ; WDT
jmp TIMER2_COMPA_vect   ; TIM2_COMPA
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


; ============================================
;   INITIALIZE
; ============================================
reset: 
    INIT_STACK_POINTER

    SET_CLK_PRESCALER

    ;initialize 8-bit timer - to be used later
init_timer2:
    ldi tmp_reg0, (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20)
    sts TCCR2B, tmp_reg0
    
    ldi tmp_reg0, $FA ;250
    sts OCR2A, tmp_reg0

    ldi tmp_reg0, (1 << OCIE2A)
    sts TIMSK2, tmp_reg0
    
    ;initialize ADC
    ldi tmp_reg0, (1 << REFS0);
    sts ADMUX, tmp_reg0
    ldi tmp_reg0, (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1)
    sts ADCSRA, tmp_reg0

init_sleep:
    ;using Power-save Mode
    ldi tmp_reg0, (1 << SM1) | (1 << SM0) | (1 << SE)
    sts SMCR, tmp_reg0



init_ports:
	ldi tmp_reg0, (1 << THRESHOLD_LED) | (1 << DEBUG_LED)
	out DDRB, tmp_reg0

init_PCINT0_vect:
    ldi tmp_reg0, ~(1 << SYS_RESET_PIN)
    sts DDRB, tmp_reg0
    ldi tmp_reg0, (1 << SYS_RESET_PIN)
    sts PORTB, tmp_reg0

    ldi tmp_reg0, (1 << PCIE0)
    sts PCICR, tmp_reg0
    ldi tmp_reg0, (1 << PCINT4)
    sts PCMSK0, tmp_reg0

    sei

; ============================================
;    MAIN LOOP
; ============================================
main:
    rcall adc_read
    
    ;handling the ADC result
    ldi tmp_reg0, ADC_THRESHOLD
;if ADCL < THRESHOLD
    cp adc_reg_low, tmp_reg0
    brlo led_off
;else
    sbi PORTB, THRESHOLD_LED      ;set bit high
    sbis PORTB,  THRESHOLD_LED     ;always skip next instruction
led_off:
    cbi PORTB, THRESHOLD_LED      ;set bit low

    sleep ;10 ms
	rjmp main


; ============================================
;   SUBROUTINE: delay_10_us
; ============================================
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
    ldi tmp_reg0, COUNTER_DELAY_10_US     ;1 cycle
delay_10_us_loop:
    dec tmp_reg0                          ;1 cycle
    brne delay_10_us_loop                  ;3 * 23 + 1
    nop                                     ;1 cycle
    ret                                     ;4 cycles
    

; ============================================
;   SUBROUTINE: adc_read
; ============================================
adc_read:
    ldi tmp_reg0, (1 << MUX0)       ;channel 0
    lds tmp_reg1, ADMUX
    or tmp_reg0, tmp_reg1
    sts ADMUX, tmp_reg0

    ldi tmp_reg0, (1 << ADSC)       ;start conversion
    lds tmp_reg1, ADCSRA
    or tmp_reg0, tmp_reg1
    sts ADCSRA, tmp_reg0

wait_until_conversion_done:
    lds tmp_reg0, ADCSRA
    cpi tmp_reg0, ADSC
    breq wait_until_conversion_done
    
    lds adc_reg_low, ADCL
    lds adc_reg_high, ADCH
    ret
    

; ============================================
;   SUBROUTINE: lcd_execute_instruction
; ============================================
lcd_execute_instruction:
    push tmp_reg0
    ldi tmp_reg0, (1  << EN)
    sts CONTROL_LINES, tmp_reg0
    nop
    nop
    andi tmp_reg0, ~(1 << EN)
    sts CONTROL_LINES, tmp_reg0
    pop tmp_reg0
    ret


; ============================================
;   INTERRUPT SERVICES
; ============================================
;To be used for watchdog system reset
PCINT0_vect:
    ;check if SYS_RESET_PIN is pressed
    sbic PINB, SYS_RESET_PIN       ;falling edge
    nop ;debug message: PCINT0
    reti

    
;used for waking up the processor
TIMER2_COMPA_vect:
    TOGGLE_BIT DEBUG_LED, PINB, PORTB
    reti


.exit
