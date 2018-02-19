;******************************************************************************;
;   HEADER FILES
;******************************************************************************;
.nolist
.include "lcd.asm"
.list

;******************************************************************************;
;   HARDWARE INFORMATION
;******************************************************************************;
;JHD 659 M10 LCD display.


;******************************************************************************;
;   PORTS AND PINS
;******************************************************************************;


;******************************************************************************;
;   CONSTANTS
;******************************************************************************;

;******************************************************************************;
; - EN (enable) control line informs the LCD, that data is being sent.
;   Set to HIGH, wait required time, then set LOW to execute instruction.
; - RS (register select), set LOW to treat data as command/special
;   instruction, HIGH treats data as text.
; - RW (read/write), LOW to write on LCD, HIGH to read LCD.
;******************************************************************************;

.equ EN = PINB0
.equ RW = PINB3
.equ RS = PINB2
.equ DATA_LINES = PORTD
.equ DATA_DIRECTION = DDRD
.equ CONTROL_LINES = PORTB
.equ CONTROL_DIRECTION = DDRB

.equ FIRST_LINE_ADDRESS = 0x80
.equ SECOND_LINE_ADDRESS = 0xC0
.equ THIRD_LINE_ADDRESS = 0x94
.equ FOURTH_LINE_ADDRESS = 0xD4

.equ CLEAR_DISPLAY = 0x01
.equ SET_4_BIT = 0x28
.equ SET_8_BIT = 0x38
.equ DISPLAY_ON_CURSOR_OFF = 0x0C
.equ DISPLAY_ON_CURSOR_ON = 0x0E
.equ DISPLAY_ON_CURSOR_BLINK = 0x0F


;******************************************************************************;
;   FIXED + DERIVED CONSTANTS
;******************************************************************************;


;******************************************************************************;
;   REGISTER DEFINITIONS
;******************************************************************************;


;******************************************************************************;
;   SRAM DEFINITIONS
;******************************************************************************;


;******************************************************************************;
;   MACROS
;******************************************************************************;
.listmac ;list macros in .lst file


.exit
