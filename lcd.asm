;******************************************************************************;
;   HEADER FILES
;******************************************************************************;


.cseg
.org $0095
 

;******************************************************************************;
; SUBROUTINE: lcd_execute_instruction
; Registers used: tmp_reg0
; Description: Sets EN bit HIGH, then after few cycles LOW to execute the given
; instruction.
;******************************************************************************;
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
    andi tmp_reg0, ~(1 << EN)
    out CONTROL_LINES, tmp_reg0

    pop tmp_reg0
    ret

    
;******************************************************************************;
; SUBROUTINE: lcd_wait_if_busy
; Registers used: tmp_reg0
; Description:
;******************************************************************************;
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
    
    
;******************************************************************************;
; SUBROUTINE: lcd_send_command
; Registers used: tmp_reg0
; Description:
;******************************************************************************;
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
    
    
;******************************************************************************;
; SUBROUTINE: lcd_send_character
; Registers used: tmp_reg0
; Description:
;******************************************************************************;
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

    
;******************************************************************************;
; SUBROUTINE: lcd_init
; Registers used: tmp_reg0
; Description:
;******************************************************************************;
lcd_init:
    push tmp_reg0
    
    ;initialize pins for LCD
    ;modify instead of overwriting to avoid problems in the future
    in tmp_reg0, CONTROL_DIRECTION
    ori tmp_reg0, (1 << EN) | (1 << RW) | (1 << RS)
    out CONTROL_DIRECTION, tmp_reg0
    rcall sleep_10ms

    ldi param_reg0, SET_8_BIT
    rcall lcd_send_command
    rcall sleep_10ms

    ldi param_reg0, DISPLAY_ON_CURSOR_OFF
    rcall lcd_send_command
    rcall sleep_10ms

    ldi param_reg0, CLEAR_DISPLAY
    rcall lcd_send_command
    rcall sleep_10ms
    
    pop tmp_reg0
    ret

    
;******************************************************************************;
;   INTERRUPT SERVICE ROUTINES
;******************************************************************************;


.exit
