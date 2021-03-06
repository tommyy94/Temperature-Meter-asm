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

.equ FIRST_LINE_ADDRESS = $80
.equ SECOND_LINE_ADDRESS = $C0
.equ THIRD_LINE_ADDRESS = $94
.equ FOURTH_LINE_ADDRESS = $D4

.equ BUSY_FLAG = $80

.equ FIRST_ROW_POS = 0
.equ SECOND_ROW_POS= 64
.equ THIRD_ROW_POS = 32
.equ FOURTH_ROW_POS = 128

.equ CLEAR_DISPLAY = $01
.equ SET_4_BIT = $28
.equ SET_8_BIT = $38
.equ DISPLAY_ON_CURSOR_OFF = $0C
.equ DISPLAY_ON_CURSOR_ON = $0E
.equ DISPLAY_ON_CURSOR_BLINK = $0F


;******************************************************************************;
;   FIXED + DERIVED CONSTANTS
;******************************************************************************;
.equ FIRST_ROW = 0
.equ SECOND_ROW = 1
.equ THIRD_ROW = 2
.equ FOURTH_ROW = 3

.equ FIRST_COLUMN = 0
.equ SECOND_COLUMN = 1
.equ THIRD_COLUMN = 2
.equ FOURTH_COLUMN = 3
.equ FIFTH_COLUMN = 4
.equ SIXTH_COLUMN = 5
.equ SEVENTH_COLUMN = 6
.equ EIGHT_COLUMN = 7
.equ NINTH_COLUMN = 8
.equ TENTH_COLUMN = 9
.equ ELEVENTH_COLUMN = 10
.equ TWELFTH_COLUMN = 11
.equ THIRTEENTH_COLUMN = 12
.equ FOURTEENTH_COLUMN = 13
.equ FIFTEENTH_COLUMN = 14
.equ SIXTEENTH_COLUMN = 15


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
