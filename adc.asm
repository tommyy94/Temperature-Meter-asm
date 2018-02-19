;******************************************************************************;
;   HEADER FILES
;******************************************************************************;


;******************************************************************************;
; SUBROUTINE: adc_read
; Registers used: tmp_reg0, pointer_reg_low, pointer_reg_high
; Description: Reads ADC value and places the result in 16-bit pointer register.
;******************************************************************************;
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
    
    
;******************************************************************************;
;   INTERRUPT SERVICE ROUTINES
;******************************************************************************;


.exit
