
;%include "pit.inc"
;%include "rtc.inc"

%macro TIMER_read	1
PIT_write_command	0x06
PIT_read_counter	PIT_TIMER_PORT
mov ah, al
PIT_read_counter	PIT_TIMER_PORT
xchg al, ah
mov word [%1], ax
%endmacro

%macro TIMER_pit_msdelay		1
xor ebx, ebx
mov ebx, %1
mov eax, X_PIT_FREQUENCY
mul ebx 
xchg ebx, eax
mov eax, [TIMER_PIT_TICKS]
add eax, ebx
xchg ebx, eax
%%check_pit:
mov eax, [TIMER_PIT_TICKS]
TIMER_read	TIMER_PIT_COUNTER
add eax, dword [TIMER_PIT_COUNTER]
cmp eax, ebx
jl %%check_pit
%endmacro

%macro TIMER_rtc_msdelay		1
mov eax, [TIMER_RTC_TICKS]
xor ebx, ebx
mov ebx, %1
add eax, ebx
xchg eax, ebx
;mov ebx, 1024
;mul ebx
;RTC_enable
%%check_rtc:
mov eax, [TIMER_RTC_TICKS]
cmp eax, ebx
jl %%check_rtc  
;RTC_disable
%endmacro

TIMER_PIT_COUNTER	DW	0
TIMER_PIT_TICKS		DD	0
TIMER_RTC_TICKS		DD	0
