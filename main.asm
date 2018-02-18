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


; ============================================
;   INITIALIZE
; ============================================
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

init_adc:
    ;using separate voltage source in ADC
    ldi tmp_reg0, (1 << REFS0)
    sts ADMUX, tmp_reg0

    ;set division factor between system clock and the ADC to 64, so:
    ;8MHz/64 = 125kHz
    ldi tmp_reg0, (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1)
    sts ADCSRA, tmp_reg0

init_sleep:
    ;using Power-save Mode
    ldi tmp_reg0, (1 << SM1) | (1 << SM0) | (1 << SE)
    sts SMCR, tmp_reg0

init_ports:
	ldi tmp_reg0, (1 << YELLOW_LED) | (1 << GREEN_LED)
	out DDRB, tmp_reg0

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

    sleep

    sei

; ============================================
;    MAIN LOOP
; ============================================
main:
    rcall adc_read
    rcall lcd_send_character
    sleep ;10 ms
    TOGGLE_BIT YELLOW_LED, PINB, PORTB
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
    ldi tmp_reg0, COUNTER_DELAY_10_US       ;1 cycle
delay_10_us_loop:
    dec tmp_reg0                            ;1 cycle
    brne delay_10_us_loop                   ;3 * 23 + 1
    nop                                     ;1 cycle
    ret                                     ;4 cycles
    

; ============================================
;   SUBROUTINE: adc_read
; ============================================
adc_read:
    ;using channel 0
    lds tmp_reg0, ADMUX
    ori tmp_reg0, (1 << MUX0)
    sts ADMUX, tmp_reg0

    ;start conversion
    lds tmp_reg0, ADCSRA
    ori tmp_reg0, (1 << ADSC)
    sts ADCSRA, tmp_reg0

wait_until_conversion_done:
    ;ADSC bit is turned off when conversion is done
    lds tmp_reg0, ADCSRA
    cpi tmp_reg0, ADSC
    breq wait_until_conversion_done

    ;load ADC result to 16-bit register
    lds pointer_reg_low, ADCL
    lds pointer_reg_high, ADCH
    ret
    

; ============================================
;   SUBROUTINE: lcd_execute_instruction
; ============================================
lcd_execute_instruction:
    push tmp_reg0

    ;set EN HIGH
    in tmp_reg0, CONTROL_LINES
    ori tmp_reg0, (1  << EN)
    out CONTROL_LINES, tmp_reg0
    ;wait for 2 cycles
    nop
    nop
    ;set EN LOW to execute given instruction
    in tmp_reg0, CONTROL_LINES
    andi tmp_reg0, ~(1 << EN)
    out CONTROL_LINES, tmp_reg0

    pop tmp_reg0
    ret

    
; ============================================
;   SUBROUTINE: lcd_wait_if_busy
; ============================================
lcd_wait_if_busy:
    push tmp_reg0
    
    ;read from data port
    andi tmp_reg0, ~$FF
    out DATA_DIRECTION, tmp_reg0

    ;command mode, read mode
    in tmp_reg0, CONTROL_LINES
    andi tmp_reg0, ~(1 << RS)
    ori tmp_reg0, (1 << RW)
    out CONTROL_LINES, tmp_reg0

keep_waiting:
    rcall lcd_execute_instruction
    in tmp_reg0, DATA_LINES
    cpi tmp_reg0, FIRST_LINE_ADDRESS
    brsh keep_waiting
    
    ;write to data port
    ori tmp_reg0, $FF
    out DATA_DIRECTION, tmp_reg0

    ;50 us delay done efficiently
    rcall delay_10_us
    rcall delay_10_us
    rcall delay_10_us
    rcall delay_10_us
    rcall delay_10_us

    pop tmp_reg0
    ret
    

; ============================================
;   SUBROUTINE: lcd_send_command
; ============================================
lcd_send_command:
    push tmp_reg0

    rcall lcd_wait_if_busy
    
    ;overwrite whole register
    out DATA_LINES, param_reg0 ;routine parameter

    ;write mode, command mode
    in tmp_reg0, CONTROL_LINES
    andi tmp_reg0, ~((1 << RW) | (1 << RS))
    out CONTROL_LINES, tmp_reg0

    ;send command here
    rcall lcd_execute_instruction

    ;clear data lines
    andi tmp_reg0, ~$FF
    out DATA_LINES, tmp_reg0

    pop tmp_reg0
    ret
    

; ============================================
;   SUBROUTINE: lcd_send_character
; ============================================
lcd_send_character:
    push tmp_reg0

    rcall lcd_wait_if_busy
    
    ;load value to datalines, so it can be sent forward
    ldi tmp_reg0, 'A'
    out DATA_LINES, tmp_reg0
    
    ;read mode, text mode
    in tmp_reg0, CONTROL_LINES
    andi tmp_reg0, ~(1 << RW)
    ori tmp_reg0, (1 << RS)
    out CONTROL_LINES, tmp_reg0

    ;send character here
    rcall lcd_execute_instruction

     ;clear data lines
    andi tmp_reg0, ~$FF
    out DATA_LINES, tmp_reg0

    ;delay needed after sending character
    rcall delay_10_us

    pop tmp_reg0
    ret


; ============================================
;   SUBROUTINE: lcd_init
; ============================================
lcd_init:
    push tmp_reg0
    
    ;initialize pins for LCD
    ;modify instead of overwriting to avoid problems in the future
    in tmp_reg0, CONTROL_DIRECTION
    ori tmp_reg0, (1 << EN) | (1 << RW) | (1 << RS)
    out CONTROL_DIRECTION, tmp_reg0
    sleep

    ldi param_reg0, SET_8_BIT
    rcall lcd_send_command
    sleep

    ldi param_reg0, DISPLAY_ON_CURSOR_OFF
    rcall lcd_send_command
    sleep

    ldi param_reg0, CLEAR_DISPLAY
    rcall lcd_send_command
    sleep
    
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


TIM2_COMPA2_vect:
    TOGGLE_BIT GREEN_LED, PINB, PORTB
    nop ;used to wake up from sleep
    reti


.exit
