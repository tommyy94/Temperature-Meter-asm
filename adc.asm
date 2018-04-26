;******************************************************************************;
;   HEADER FILES
;******************************************************************************;


;******************************************************************************;
; SUBROUTINE: adc_read
; Registers used: r16, r17
; Description: Reads ADC value and places the result in r17:r16.
;******************************************************************************;
adc_read:
    ;using channel 0 by default

    ;start conversion
    lds r16, ADCSRA
    ori r16, (1 << ADSC)
    sts ADCSRA, r16

wait_until_conversion_done:
    ;ADSC bit is turned off when conversion is done
    lds r16, ADCSRA
    sbrc r16, ADSC
    rjmp wait_until_conversion_done

    ;load ADC result to 16-bit register
    lds r17, ADCH
    lds r16, ADCL

    ret
    
    
;******************************************************************************;
;   INTERRUPT SERVICE ROUTINES
;******************************************************************************;


.exit
