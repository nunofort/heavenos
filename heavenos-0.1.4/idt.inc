;Figure 9-3.  80306 IDT Gate Descriptors
;
;                                80386 TASK GATE
;   31                23                15                7                0
;  ÉÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÑÍÍÍÑÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ»
;  º±±±±±±±±±±±±±(NOT USED)±±±±±±±±±±±±³ P ³DPL³0 0 1 0 1³±±±(NOT USED)±±±±º4
;  ÇÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÅÄÄÄÁÄÄÄÁÄÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ¶
;  º             SELECTOR              ³±±±±±±±±±±±±±(NOT USED)±±±±±±±±±±±±º0
;  ÈÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ¼

;                                80386 INTERRUPT GATE
;   31                23                15                7                0
;  ÉÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÑÍÍÍÑÍÍÍÍÍÍÍÍÍØÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍ»
;  º           OFFSET 31..16           ³ P ³DPL³0 1 1 1 0³0 0 0³(NOT USED) º4
;  ÇÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÅÄÄÄÁÄÄÄÁÄÄÄÄÄÄÄÄÄÁÄÄÄÄÄÁÄÄÄÄÄÄÄÄÄÄÄ¶
;  º             SELECTOR              ³           OFFSET 15..0            º0
;  ÈÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ¼

;                                80386 TRAP GATE
;   31                23                15                7                0
;  ÉÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÑÍÍÍÑÍÍÍÍÍÍÍÍÍØÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍ»
;  º          OFFSET 31..16            ³ P ³DPL³0 1 1 1 1³0 0 0³(NOT USED) º4
;  ÇÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÅÄÄÄÁÄÄÄÁÄÄÄÄÄÄÄÄÄÁÄÄÄÄÄÁÄÄÄÄÄÄÄÄÄÄÄ¶
;  º             SELECTOR              ³           OFFSET 15..0            º0
;  ÈÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍØÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ¼
;
;==============================================================================

;%include "pic.inc"
;%include "floppy.inc"

fill_idt:
mov edi, idt
mov esi, int_address
next_int:
mov eax, dword [es:esi]
mov word [es:edi], ax
add edi, 6
shr eax, 16
mov word [es:edi], ax
add edi, 2  
add esi, 4 
cmp esi, int_address_size
jne next_int
retf

	
idt_ptr:
	 	dw	idt_size-1		; IDT Size
		dd	idt			; IDT Address	
idt:
;==============================================================================
; => HARDWARE EXCEPTIONS
;==============================================================================
;------------------------------------------------------------------------------
; [0x00 * 0x08] EXCEPTION 0 - Divide error
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x01 * 0x08] EXCEPTION 1 - Debug exceptions
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x02 * 0x08] EXCEPTION 2 - Nonmaskable interrupt 
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x03 * 0x08] EXCEPTION 3 - Breakpoint (one-byte INT 3 instruction) 
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x04 * 0x08] EXCEPTION 4 - Overflow (INTO instruction) 
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x05 * 0x08] EXCEPTION 5 - Bounds check (BOUND instruction)
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x06 * 0x08] EXCEPTION 6 - Invalid opcode
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x07 * 0x08] EXCEPTION 7 - Coprocessor not available
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x08 * 0x08] EXCEPTION 8 - Double fault
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x09 * 0x08] EXCEPTION 9 - (reserved)
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x0A * 0x08] EXCEPTION 10 - Invalid TSS
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x0B * 0x08] EXCEPTION 11 - Segment not present
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x0C * 0x08] EXCEPTION 12 - Stack exception
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x0D * 0x08] EXCEPTION 13 - General protection
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x0E * 0x08] EXCEPTION 14 - Page fault
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x0F * 0x08] EXCEPTION 15 - (reserved)
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x10 * 0x08] EXCEPTION 16 - Coprocessor error
;------------------------------------------------------------------------------
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x11 * 0x08] EXCEPTION 17
;------------------------------------------------------------------------------
;17-31         (reserved)

times (15 * 0x08)	DB 0
;==============================================================================
; => HARDWARE INTERRUPTS
;==============================================================================
;------------------------------------------------------------------------------
; [0x20 * 0x08] IRQ0 System Timer - INT 08h
;------------------------------------------------------------------------------ 
		dw	0			; OFFSET Address [0..15] 
		dw	sys_tss-gdt		; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10000101b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]	
;------------------------------------------------------------------------------
; [0x21 * 0x08] IRQ1 Keyboard - INT 09h
;------------------------------------------------------------------------------ 		
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x22 * 0x08] IRQ2 Slave PIC - INT 0Ah
;------------------------------------------------------------------------------ 		
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x23 * 0x08] IRQ3 Second Serial Interface - INT 0Bh
;------------------------------------------------------------------------------ 		
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x24 * 0x08] IRQ4 First Serial Interface - INT 0Ch
;------------------------------------------------------------------------------ 		
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x25 * 0x08] IRQ5 Second Parallel Interface - INT 0Dh
;------------------------------------------------------------------------------ 		
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x26 * 0x08] IRQ6 Floppy Drive Controller - INT 0Eh
;------------------------------------------------------------------------------ 		
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x27 * 0x08] IRQ7 First Parallel Interface - INT 0Fh
;------------------------------------------------------------------------------ 		
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;------------------------------------------------------------------------------
; [0x28 * 0x08] IRQ8 Real Time Clock - INT 10h
;------------------------------------------------------------------------------ 		
		dw	0			; OFFSET Address [0..15] 
		dw	0x0008			; SELECTOR of GDT	
		db	0			; (NOT USED)
		db	10001110b		; P=1 DPL=00 01110
		dw	0			; OFFSET Address [16..31]
;==============================================================================	
; => SOFTWARE INTERRUPTS
;==============================================================================
;------------------------------------------------------------------------------
;
;------------------------------------------------------------------------------
; MAX. INT'S = 256
idt_size	equ	$-idt
int_address:	
		dd	exception_0		; EXCEPTION 0
		dd	exception_1		; EXCEPTION 1
		dd	exception_2		; EXCEPTION 2
		dd	exception_3		; EXCEPTION 3
		dd	exception_4		; EXCEPTION 4
		dd	exception_5		; EXCEPTION 5
		dd	exception_6		; EXCEPTION 6
		dd	exception_7		; EXCEPTION 7
		dd	exception_8		; EXCEPTION 8
		dd	exception_9		; EXCEPTION 9
		dd	exception_10		; EXCEPTION 10
		dd	exception_11		; EXCEPTION 11
		dd	exception_12		; EXCEPTION 12
		dd	exception_13		; EXCEPTION 13
		dd	exception_14		; EXCEPTION 14
		dd	exception_15		; EXCEPTION 15
		dd	exception_16		; EXCEPTION 16
times 15 	dd	0			; EXCEPTION 17-31
		dd	0			; IRQ0 timer_int as task gate!
		dd	keyboard_int		; IRQ1
		dd	0			; IRQ2
		dd	0			; IRQ3
		dd	0			; IRQ4
		dd	0			; IRQ5
		dd	floppy_int		; IRQ6
		dd	0			; IRQ7
		dd	rtc_int			; IRQ8
int_address_size:
;==============================================================================
; INT ROUTINES
;==============================================================================
;   0   IRQ0 System Timer | for MULTI-TASKING (PIT at 50 ticks/s)
;   1   IRQ1 Keyboard
;   2   IRQ0 Real time clock | for ms DELAYs ( IRQ8 )
;   3   IRQ1 Free
;   4   IRQ2 Free
;   5   IRQ3 Free
;   6   IRQ4 Free
;   7   IRQ5 Math coprocessor
;   8   IRQ6 Hard disk controller
;   9   IRQ7 Free
;   10  IRQ2 Slave PIC
;   11  IRQ3 Second serial interface
;   12  IRQ4 First serial interface
;   13  IRQ5 Second parallel interface
;   14  IRQ6 Floppy drive controller
;   15  IRQ7 First parallel interface
;
; IRQ Routing Procedure:
; Ex: Driver de SOM (placa de som), depois de o IRQ estar colocado para a placa de SOM,
; vai colocar no IRQ da IDT o address da ROTINA(INT) de som.
; Software Task(CALL) Gates for kernel modules(libraries) accessing
; A CALL GATE on GDT can have level 0 access
; INTERFACE for drivers and application resource access


%macro save_regs	0
mov [REGISTER_EAX], eax
mov [REGISTER_EBX], ebx
mov [REGISTER_ECX], ecx
mov [REGISTER_EDX], edx
mov [REGISTER_EDI], edi
mov [REGISTER_ESI], esi
mov [REGISTER_ESP], esp
mov [REGISTER_EBP], ebp
mov dword [REGISTER_EIP], $ 
;mov [REGISTER_EFLAGS], eflags
mov [REGISTER_ES], es
mov [REGISTER_DS], ds
mov [REGISTER_SS], ss
mov [REGISTER_FS], fs
mov [REGISTER_GS], gs
%endmacro

%macro show_regs	0
number_to_text [REGISTER_EAX], 16, mensage_eax+13
number_to_text [REGISTER_EBX], 16, mensage_ebx+13
number_to_text [REGISTER_ECX], 16, mensage_ecx+13
number_to_text [REGISTER_EDX], 16, mensage_edx+13
number_to_text [REGISTER_EDI], 16, mensage_edi+13
number_to_text [REGISTER_ESI], 16, mensage_esi+13
number_to_text [REGISTER_ESP], 16, mensage_esp+13
number_to_text [REGISTER_EBP], 16, mensage_ebp+13
number_to_text [REGISTER_EIP], 16, mensage_eip+13
number_to_text [REGISTER_ES], 16, mensage_es+8
number_to_text [REGISTER_DS], 16, mensage_ds+8
number_to_text [REGISTER_SS], 16, mensage_ss+8
number_to_text [REGISTER_FS], 16, mensage_fs+8
number_to_text [REGISTER_GS], 16, mensage_gs+8
VIDEO_TEXTMODE_write_mensage            0x03, 0x01, 0x0F, mensage_eax
VIDEO_TEXTMODE_write_mensage            0x04, 0x01, 0x0F, mensage_ebx
VIDEO_TEXTMODE_write_mensage            0x05, 0x01, 0x0F, mensage_ecx
VIDEO_TEXTMODE_write_mensage            0x06, 0x01, 0x0F, mensage_edx
VIDEO_TEXTMODE_write_mensage            0x07, 0x01, 0x0F, mensage_edi
VIDEO_TEXTMODE_write_mensage            0x08, 0x01, 0x0F, mensage_esi
VIDEO_TEXTMODE_write_mensage            0x09, 0x01, 0x0F, mensage_esp
VIDEO_TEXTMODE_write_mensage            0x0A, 0x01, 0x0F, mensage_ebp
VIDEO_TEXTMODE_write_mensage            0x0B, 0x01, 0x0F, mensage_eip
VIDEO_TEXTMODE_write_mensage            0x0C, 0x01, 0x0F, mensage_es
VIDEO_TEXTMODE_write_mensage            0x0D, 0x01, 0x0F, mensage_ds
VIDEO_TEXTMODE_write_mensage            0x0E, 0x01, 0x0F, mensage_ss
VIDEO_TEXTMODE_write_mensage            0x0F, 0x01, 0x0F, mensage_fs
VIDEO_TEXTMODE_write_mensage            0x10, 0x01, 0x0F, mensage_gs
%endmacro

exception_0:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_0
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_1:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_1
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_2:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_2
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_3:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_3
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_4:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_4
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_5:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_5
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_6:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_6
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_7:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_7
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_8:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_8
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_9:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_9
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_10:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_10
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_11:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_11
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_12:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_12
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_13:
save_regs
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_13
show_regs
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_14:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_14
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_15:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_15
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

exception_16:
cli
pusha
call dword 0x0008:VIDEO_TEXTMODE_clear_screen
VIDEO_TEXTMODE_write_mensage            0x01, 0x01, 0x0F, exception_msg_16
VIDEO_TEXTMODE_write_mensage            0x16, 0x01, 0x03, exception_reboot_msg 

KEYBOARD_direct_check_key	KEYBOARD_ENTER_KEY
jmp dword 0x0008:KEYBOARD_system_reset
; recover from error
PIC_eoi_cmd
popa
sti
iretd

timer_int:
cli
pusha
add dword [TIMER_PIT_TICKS], 65536 
PIC_eoi_cmd
popa
; GET FROM STACK 
; EIP on the 4-byte				; updated
; CS on the 4-byte, the top 2 are discarded 	; the same
; EFLAGS on the 4-byte			    	; the same
sti
iretd

keyboard_int:
cli
pusha
KEYBOARD_write	KEYBOARD_COMMAND_PORT, 0xAD
in al, KEYBOARD_DATA_PORT	; Keyboard Scan Code
mov [KEYBOARD_KEY], al
KEYBOARD_write	KEYBOARD_COMMAND_PORT, 0xAE
PIC_eoi_cmd
popa
sti
iretd

floppy_int:
cli
pusha
; End of Floppy Operation
mov byte [FLOPPY_INTERRUPT], 1
PIC_eoi_cmd
popa
sti
iretd

rtc_int:
cli
pusha
inc dword [TIMER_RTC_TICKS]
cmp dword [TIMER_RTC_TICKS],0xFFFFFFFF
jne skip_reset
mov dword [TIMER_RTC_TICKS], 0
skip_reset:
;;; CHECK for RTC Interrupt (periodic int,alarm time reached,time changed)
RTC_read_register       0x0C, RTC_VALUE
PIC_eoi_cmd
popa
sti
iretd

;========================================================================;
exception_msg_0		DB 'Exception 0 : Divide error',0
exception_msg_1		DB 'Exception 1 : Debug exceptions',0
exception_msg_2		DB 'Exception 2 : Nonmaskable interrupt',0
exception_msg_3		DB 'Exception 3 : Breakpoint (one-byte INT 3 instruction)',0
exception_msg_4		DB 'Exception 4 : Overflow (INTO instruction)',0
exception_msg_5		DB 'Exception 5 : Bounds check (BOUND instruction)',0
exception_msg_6		DB 'Exception 6 : Invalid opcode',0
exception_msg_7		DB 'Exception 7 : Coprocessor not available',0
exception_msg_8		DB 'Exception 8 : Double fault',0
exception_msg_9		DB 'Exception 9 : (reserved)',0
exception_msg_10	DB 'Exception 10 : Invalid TSS',0
exception_msg_11	DB 'Exception 11 : Segment not present',0
exception_msg_12	DB 'Exception 12 : Stack exception',0
exception_msg_13	DB 'Exception 13 : General protection',0
exception_msg_14	DB 'Exception 14 : Page fault',0
exception_msg_15	DB 'Exception 15 : (reserved)',0
exception_msg_16	DB 'Exception 16 : Coprocessor error',0
exception_reboot_msg	DB 'PRESS [ENTER] FOR REBOOT...',0
REGISTER_EAX		DD 0
REGISTER_EBX		DD 0
REGISTER_ECX		DD 0
REGISTER_EDX		DD 0
REGISTER_EDI		DD 0
REGISTER_ESI		DD 0
REGISTER_ESP		DD 0
REGISTER_EBP		DD 0
REGISTER_EIP		DD 0
;REGISTER_EFLAGS		DD 0
REGISTER_ES		DW 0
REGISTER_DS		DW 0
REGISTER_SS		DW 0
REGISTER_FS		DW 0				
REGISTER_GS		DW 0
mensage_eax		DB 'EAX =         h',0
mensage_ebx		DB 'EBX =         h',0
mensage_ecx		DB 'ECX =         h',0
mensage_edx		DB 'EDX =         h',0
mensage_edi		DB 'EDI =         h',0
mensage_esi		DB 'ESI =         h',0
mensage_esp		DB 'ESP =         h',0
mensage_ebp		DB 'EBP =         h',0
mensage_eip		DB 'EIP =         h',0
;mensage_eflags		DB 'EFLAGS =         h',0
mensage_es		DB 'ES =     h',0
mensage_ds		DB 'DS =     h',0
mensage_ss		DB 'SS =     h',0
mensage_fs		DB 'FS =     h',0
mensage_gs		DB 'GS =     h',0

