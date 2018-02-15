;**********************************************************************
; Copyright 2006, Nuno Miguel (codingheaven@gmail.com) 
; All rights reserved. 
;
; See the LICENSE file
;
; The origin of this software must not be misrepresented, either by
; explicit claim or by omission.  Since few users ever read sources,
; credits must appear in the documentation.
;
; CODE IMPROVEMENTS by:
;	Nuno Miguel (codingheaven@gmail.com)	
;
;       
;
;
;
;***********************************************************************


;=============================================================;
; HeavenOS Micro-Kernel V0.1.4
; In the future, Version Number should be only on VERSION file 							               ;
;=============================================================;
;    500 - 9FFFF Free Memory < 1MB (Conventional 640K)                        ;
; 100000 - ?     Free Memory >= 1MB (Extended)                                ;
;-----------------------------------------------------------------------------;

[BITS 16]
;[ORG 0x00000500]
;[ORG 0x00010000]
[ORG 0x00001000]
;-----------------------------------------------------------------------------;
; 16 BITS CODE                                                                ;
;-----------------------------------------------------------------------------;

; set ES to video address
mov ax, 0xB800
mov es, ax

;;; RM_Clear_Screen
mov cx, 0x0fa0
mov di, 0
next_char0:
mov dword [es:di], 0
add di, 4
cmp di, cx
jne next_char0

;;; RM_Write_Message
mov ax, 0xB800
mov es, ax
;; RM_Write_Boot_Mensage
mov di, 160*0x02 + 2*0x01
mov si, kernel_msg
mov ah, 0x03
next_char1:
mov al, byte [si]
mov word [es:di], ax
times 2 inc di
inc si
cmp byte [si],0
jne next_char1

check_keyboard:
in   al, 0x60                
cmp  al, 0x1C    	; check ENTER key           
jne  check_keyboard      

;---------------------------------------------------------------------------;
; INITIALIZATION STAGE FOR THE 32-BIT PM SWITCH                             ;
;---------------------------------------------------------------------------;
cli     ; disable interrupts

; Enable A20 on PS/2 compatibles
;in      al,0x92
;or      al,2
;out     0x92,al

; Enable A20 on AT compatibles > 1MB
mov     al,0xD1
out     0x64,al
mov     al,0xDF
out     0x60,al

lgdt [gdt_ptr]			; Load GDT into GDTR

; switch to PM
mov     eax, cr0                ; Get Control Register 0
or      eax, 0x01               ; set PE bit (bit 0) in (e)ax
mov     cr0, eax                ; Switch to Protected Mode

; setting regs
mov ax, 0x0010
mov es, ax      ; extra segment
mov ds, ax      ; data segment
mov ss, ax      ; stack segment -> grows down in memory
mov fs, ax
mov gs, ax
mov esp, 0x5000         ; stack pointer
mov ebp, 0x5000         ; base pointer

jmp dword 0x0008:kernel_start 

;---------------------------------------------------------------------------;
; 32-BIT CODE                                                              ;
;---------------------------------------------------------------------------;
[BITS 32]

%include "cpu.inc"

%include "util.inc"
%include "dma.inc"
%include "pic.inc"
%include "floppy.inc"
%include "pit.inc"
%include "cmos.inc"
%include "rtc.inc"
%include "timer.inc"
%include "video.inc"
%include "keyboard.inc"

%include "gdt.inc"
%include "idt.inc";

%include "memory.inc"
%include "bios.inc"
%include "vesa.inc"
%include "pci.inc"
%include "isapnp.inc"
;%include "shellcmds.inc"
%include "tss.inc"

%include "filesystem.inc"


;---------------------------------------------------------------------------;
; KERNEL MAIN CODE                                                          ;
;---------------------------------------------------------------------------;
kernel_start:

call dword 0x0008:fill_idt	; FILL IDT with INT Addresses
lidt [idt_ptr]                  ; Load IDT into IDTR

call dword 0x0008:PIC_setup	; SETUP PIC

PIC_master_set_irqs     10111001b ; Disable IRQ0 (Timer INT) 

call dword 0x0008:PIT_setup	; SETUP PIT

KEYBOARD_init			; Initialize Keyboard

RTC_int_enable                  ; Enable RTC Interrupts

sti				; Enable Interrupts

; TESTING DEBUG ON EXCEPTION (REAL COOL! =)
;dd 0xABCD00DA			; this will run Exception 13	(GENERAL PROTECTION)

VIDEO_TEXTMODE_write_mensage 		0x03,0x01,0x07,kernel_msg
KEYBOARD_check_key 0x1C
;===============================================;
; Copying Filesystem from Floppy to Ramdisk...  ;
;===============================================;
;VIDEO_TEXTMODE_write_mensage 		0x04,0x01,0x02,copy_filesystem_msg
;FLOPPY_copy_to_address

;RAMDISK_read
;VIDEO_TEXTMODE_write_mensage            0x05,0x01,0x07,RAMDISK_BUFFER
;===============================================;
; Begin Multi-Tasking!
;===============================================;
TIMER_rtc_msdelay	2000
call dword 0x0008:VIDEO_VGA_fadeout
call dword 0x0008:VIDEO_TEXTMODE_clear_screen

call dword 0x0008:init_tss	; Initialize TSS's

mov ax, sys_tss-gdt 
ltr ax				; Load First TSS Descriptor into TR
cli
PIC_master_set_irqs     10111000b ; Enable IRQ0 (Timer INT) 
jmp dword 0x0008:sheduler

;=============================================================================;
kernel_msg	 	DB 'Loading HeavenOS Kernel V0.1.4',0
dword_number		DD 0
dword_number2		DD 0
word_number		DW 0
bytenumber		DB 0
number_string		DB '          ',0
copy_filesystem_msg	DB 'Copying Filesystem from Floppy to Ramdisk...',0
temp_string		DB '                                                                               ',0
;=============================================================================;
;times 64000-($-$$) 	DB 0
;640x1024 = max_kernel_size

