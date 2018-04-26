;******************************************************************************;
;   HEADER FILES
;******************************************************************************;


.cseg
.org $0150
 

;******************************************************************************;
; SUBROUTINE: lcd_execute_instruction
; Registers used: r16
; Description: Sets EN bit HIGH, then after 2 cycles LOW to execute the given
; instruction.
;******************************************************************************;
lcd_execute_instruction:
    push r16

    ;set EN HIGH
    in r16, CONTROL_LINES
    ori r16, (1  << EN)
    out CONTROL_LINES, r16
    ;wait for 2 cycles
    nop
    nop
    ;set EN LOW to execute given instruction
    andi r16, ~(1 << EN)
    out CONTROL_LINES, r16

    pop r16
    ret

    
;******************************************************************************;
; SUBROUTINE: lcd_wait_if_busy
; Registers used: r16
; Description: Sets command and read mode, data direction to reading.
; Stays in loop until LCD is available, sets data direction to writing.
;******************************************************************************;
lcd_wait_if_busy:
    push r16
    
    ;read from data port 
    andi r16, ~$FF
    out DATA_DIRECTION, r16

    ;command mode, read mode
    in r16, CONTROL_LINES
    andi r16, ~(1 << RS)
    ori r16, (1 << RW)
    out CONTROL_LINES, r16

keep_waiting:
    rcall lcd_execute_instruction
    in r16, DATA_LINES
    cpi r16, BUSY_FLAG
    brsh keep_waiting
    
    ;write to data port
    ori r16, $FF
    out DATA_DIRECTION, r16
    
    ;50 us delay done efficiently
    rcall delay_10_us
    rcall delay_10_us
    rcall delay_10_us
    rcall delay_10_us
    rcall delay_10_us

    pop r16
    ret
    
    
;******************************************************************************;
; SUBROUTINE: lcd_send_command
; Registers used: r16
; Description: Sends command to data lines, sets write and command mode, sends
; the command and clears the data lines.
;******************************************************************************;
lcd_send_command:
    push r16

    rcall lcd_wait_if_busy
    
    ;overwrite whole register
    out DATA_LINES, r20 ;routine parameter

    ;write mode, command mode
    in r16, CONTROL_LINES
    andi r16, ~((1 << RW) | (1 << RS))
    out CONTROL_LINES, r16

    ;send command here
    rcall lcd_execute_instruction

    ;clear data lines
    andi r16, ~$FF
    out DATA_LINES, r16

    pop r16
    ret
    
    
;******************************************************************************;
; SUBROUTINE: lcd_goto
; Registers used: r20, r21, r22
; Parameters: x & y -coordinates
; Description: Moves the LCD cursor relative to first the line address.
;******************************************************************************;
lcd_goto:
    push r16
    push r21
    push r22

    ;sum params together to get new cursor position
    ldi r20, FIRST_LINE_ADDRESS
    add r20, r21
    add r20, r22

    rcall lcd_send_command

    ;50 us delay needed
    rcall delay_10_us
    rcall delay_10_us
    rcall delay_10_us
    rcall delay_10_us
    rcall delay_10_us

    pop r22
    pop r21
    pop r16
    ret

;******************************************************************************;
; SUBROUTINE: lcd_send_character
; Registers used: r16, r20
; Description: Sends character to be sent to data lines, sets read and text
; mode, then sends it and clears data lines.
;******************************************************************************;
lcd_send_character:
    push r16

    rcall lcd_wait_if_busy
    
    ;load value to datalines, so it can be sent forward
    out DATA_LINES, r20
    
    ;read mode, text mode
    in r16, CONTROL_LINES
    andi r16, ~(1 << RW)
    ori r16, (1 << RS)
    out CONTROL_LINES, r16

    ;send character here
    rcall lcd_execute_instruction

     ;clear data lines
    andi r16, ~$FF
    out DATA_LINES, r16

    ;delay needed after sending character
    rcall delay_10_us

    pop r16
    ret

    
;******************************************************************************;
; SUBROUTINE: lcd_send_string
; Registers used: r16 (char counter), r20 (pointer to character),
; ZH:ZL (pointer to string)
; Description: Sends string of characters to data lines.
;******************************************************************************;
lcd_send_string:
    ;move cursor to given position
    rcall lcd_goto

lcd_send_char_loop:
    ;read character from table (RAM) and pass as parameter to subroutine
    ld r20, Z+ ;inc pointer for next character
    dec r16
    rcall lcd_send_character ;else send the loaded character
    cpi r16, 0 ;if loaded value is escape sequence character
    brne lcd_send_char_loop ;jump to end

    ret


;******************************************************************************;
; SUBROUTINE: lcd_init
; Registers used: r16, r20
; Description: Sets enable, register select and read/write bits in
; CONTROL_DIRECTION port. Also sets the LCD to use 8-bit mode, cursor mode and
; clears the display.
;******************************************************************************;
lcd_init:
    ;initialize pins for LCD
    ;modify instead of overwriting, because something else
    ;might use CONTROL_DIRECTION port too
    in r16, CONTROL_DIRECTION
    ori r16, (1 << EN) | (1 << RW) | (1 << RS)
    out CONTROL_DIRECTION, r16
    rcall sleep_10ms

    ldi r20, SET_8_BIT
    rcall lcd_send_command
    rcall sleep_10ms

    ldi r20, DISPLAY_ON_CURSOR_OFF
    rcall lcd_send_command
    rcall sleep_10ms

    ldi r20, CLEAR_DISPLAY
    rcall lcd_send_command
    rcall sleep_10ms
    
    ret

    
;******************************************************************************;
;   INTERRUPT SERVICE ROUTINES
;******************************************************************************;


.exit
