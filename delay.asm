;******************************************************************************;
;   HEADER FILES
;******************************************************************************;


;******************************************************************************;
; SUBROUTINE: delay_10_us
; Registers used: r16
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
    ldi r16, COUNTER_DELAY_10_US       ;1 cycle
delay_10_us_loop:
    dec r16                            ;1 cycle
    brne delay_10_us_loop                   ;3 * 23 + 1
    nop                                     ;1 cycle
    ret                                     ;4 cycles

    
;******************************************************************************;
; SUBROUTINE: sleep_10ms
; Registers used: r16
; Description: Sleeps for 10 ms. Writes Sleep Enable bit to logic one and sleep 
; until interrupt occurs, then clears Sleep Enable bit.
;******************************************************************************;
sleep_10ms:
    push r16
    
    sei ;enable interrupts so we can wake up

    ;enable sleep
    in r16, SMCR
    ori r16, (1 << SE)
    out SMCR, r16

    sleep

    ;disable sleep
    andi r16, ~(1 << SE)
    out SMCR, r16
    
    pop r16
    ret

    
;******************************************************************************;
;   INTERRUPT SERVICE ROUTINES
;******************************************************************************;
   
;******************************************************************************;
; TIM2_COMPA2_vect
; Registers used: none
; Description: Used to wake up the processor.
;******************************************************************************;
TIM2_COMPA2_vect:
    nop ;used to wake up from sleep
    reti


.exit
