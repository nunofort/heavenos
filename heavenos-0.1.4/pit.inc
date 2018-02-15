; Put IRQ0(Timer) routine on the IDT

%macro PIT_write_command	1
mov al, %1
out PIT_COMMAND_PORT, al
%endmacro

%macro PIT_read_counter		1	
jmp short $+2		; this slow I/O on fast processors
in al, %1
; return al
%endmacro

%macro PIT_write_counter	1
; al = value
jmp short $+2		; this slow I/O on fast processors
out %1, al
%endmacro

PIT_setup:
PIT_write_command       PIT_TIMER_COMMAND_WRITE_MODE_2
mov ax, 11932
PIT_write_counter       PIT_TIMER_PORT
xchg al, ah
PIT_write_counter       PIT_TIMER_PORT
retf

;=======================================================================;
; PIT-8253/8254 (Programmable Interval Timer)				;
;=======================================================================;
; 65536 ticks/cycle * 18.2 cycles/sec
; 65535,65534,65533,...1,0,65535,65534,...
SEC_PIT_FREQUENCY			equ	1193200	; Hz (ticks/second)
MSEC_PIT_FREQUENCY			equ	1193	; Hz (ticks/msecond)
X_PIT_FREQUENCY				equ	6553
; 50 ticks/s = 23864
; 100 ticks/s = 11932
PIT_COMMAND_PORT 		equ	0x43
PIT_TIMER_PORT			equ	0x40
PIT_TIMER_COMMAND_READ		equ	0x00
PIT_TIMER_COMMAND_WRITE_MODE_2	equ	0x34
PIT_TIMER_COMMAND_WRITE_MODE_3	equ	0x36

;-----------------------------------------------------------------------;
;The timers are clocked at 1,193,180Hz.
;The frequency of the interrupts is CLK / CountValue.
;The only count value for which there is an whole number of ticks
;per second is 59,659 which results in exactly 20 ticks per second.
;The possible frequency range goes from 18.2Hz - 1.29MHz.
;
; 1,193,180Hz / (65536 OR 0) = 18.206 ticks per second
; Real Time Counter (8253/8254 PIT-Programmable Interval Timer)
;
;PORT(1st PIT)  PORT(2nd PIT)   REGISTER                ACCESS_TYPE
;040h                   048h    counter0                read/write
;041h                   049h    counter1                read/write
;042h                   04ah    counter2                read/write
;043h                   04bh    ControlRegister 	write-only
;
;Control Register
;
;Bits  76   54   321    0
;    | SC | RW | MODE | BCD |
;
;SC: Selector counter
;       00=counter0     10=counter2
;       01=counter1     11=invalid on 8253, 
;			read-back command on 8254(read counter)
;RW: read/write
;       00=counter latch instruction(read counter)
;       01=read/write low counter byte only
;       10=read/write high counter byte only
;       11=read/write low counter byte first, then high counter byte
;MODE: PIT counting mode
;       000=mode0       010=mode2       100=mode4
;       001=mode1       011=mode3       101=mode5
;BCD: counting format
;       0=binary counting with 16bits (values 0h...ffffh)
;       1=counting with binary coded decimals (values 0...9999)
;------------------------------------------------------------------------;
