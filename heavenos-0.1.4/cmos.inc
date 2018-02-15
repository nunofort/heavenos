
%macro	CMOS_read_address	2
cli
;in al, 0x70     ; read old byte from port 0x70
;and al, 0xE0    ; clear the five low-order bits
;or al, %1	; write CMOS Address to al
mov al, %1
out 0x70,al     ; Wake up CMOS PORT
xor eax, eax	; clean eax 
jmp short $+2   ; Some delay...
in al,0x71      ; Reads Value.
mov %2, eax  
sti
%endmacro

; For RTC you must first read the STATUS register A, bit 7 to be 0
%macro CMOS_rtc_ready	0
%%rtc_check
CMOS_read_address               CMOS_RTC_STATUS_REGA_ADDRESS, [CMOS_TEMP_VALUE]
mov eax, [CMOS_TEMP_VALUE]
shr eax, 7
and eax, 1
cmp eax, 1
je %%rtc_check
%endmacro

;TODO: get PC information from CMOS
;===========================================================================;
; CMOS                                                                      ;
;===========================================================================;
CMOS_TEMP_VALUE			DD	0
CMOS_TIME_SECOND_ADDRESS	equ	0x00
CMOS_TIME_MINUTE_ADDRESS	equ	0x02
CMOS_TIME_HOUR_ADDRESS		equ     0x04
CMOS_DATE_DAYOFWEEK_ADDRESS	equ	0x06
CMOS_DATE_DAYOFMONTH_ADDRESS	equ	0x07
CMOS_DATE_MONTH_ADDRESS		equ	0x08
CMOS_DATE_YEAR_ADDRESS		equ	0x09
CMOS_RTC_STATUS_REGA_ADDRESS	equ	0x0A
CMOS_RTC_STATUS_REGB_ADDRESS	equ	0x0B
CMOS_RTC_STATUS_REGC_ADDRESS	equ	0x0C
CMOS_RTC_STATUS_REGD_ADDRESS	equ	0x0D
CMOS_DATE_DAYS_OF_WEEK          DB ' Mon Tue Wed Thu Fri Sat Sun'
CMOS_DATE_MONTHS_OF_YEAR        DB ' Jan Feb Mar Apr Mai Jun Jul Aug Set Oct Nov Dec'

