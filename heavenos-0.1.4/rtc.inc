
; Using CMOS Port

%macro RTC_read_register	2	
mov dx, 0x70
mov al, %1
out dx, al
jmp short $+2
mov dx, 0x71
in al, dx
mov byte [%2], al
%endmacro

%macro RTC_write_register	2
mov dx, 0x70
mov al, %1
out dx, al
jmp short $+2 
mov dx, 0x71
mov al, %2
out dx, al
%endmacro

%macro RTC_int_enable	0
;cli
;pushf
RTC_read_register 	0x0A, RTC_VALUE
and byte [RTC_VALUE], 0xF0
;;; clears bit0-bit3
or byte [RTC_VALUE], 0x06
RTC_write_register	0x0A, [RTC_VALUE]
;;; 1024 interrupts per second (0110 bit0-bit3)
RTC_read_register	0x0B, RTC_VALUE
or byte [RTC_VALUE], 0x40
RTC_write_register	0x0B, [RTC_VALUE]
;;; RTC INT Enabled (bit6)
;popf
;sti
%endmacro

%macro RTC_int_disable	0
mov al, 0x0B
out 0x70, al
jmp short $+2
in al, 0x71
and al, 0xBF
out 0x71, al
%endmacro


RTC_VALUE	DB	0
