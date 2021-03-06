;Figure 7-1.  80386 32-Bit Task State Segment
;
;      31              23              15              7             0
;     浜様様様様様様様慷様様様様様様様陵様様様様様様様慷様様様様様様僕�
;     �          I/O MAP BASE         � 0 0 0 0 0 0 0   0 0 0 0 0 0 �T�64
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳伉�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              LDT              �60
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              GS               �5C
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              FS               �58
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              DS               �54
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              SS               �50
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              CS               �4C
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              ES               �48
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                              EDI                              �44
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                              ESI                              �40
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                              EBP                              �3C
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                              ESP                              �38
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                              EBX                              �34
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                              EDX                              �30
;     麺様様様様様様様慷様様様様様様様慷様様様様様様様慷様様様様様様様�
;     �                              ECX                              �2C
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                              EAX                              �28
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                            EFLAGS                             �24
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                    INSTRUCTION POINTER (EIP)                  �20
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                          CR3  (PDPR)                          �1C
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              SS2              �18
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                             ESP2                              �14
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              SS1              �10
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                             ESP1                              �0C
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�              SS0              �8
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �                             ESP0                              �4
;     把陳陳陳陳陳陳陳田陳陳陳陳陳陳陳彡陳陳陳陳陳陳陳田陳陳陳陳陳陳陳�
;     �0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0�   BACK LINK TO PREVIOUS TSS   �0
;     藩様様様様様様様慷様様様様様様様陵様様様様様様様慷様様様様様様様�


;Figure 7-2.  TSS Descriptor for 32-bit TSS
;
;  31                23                15                7               0
; 浜様様様様様様様様慷冤冤冤冤様様様様慷冤様様冤様様様様慷様様様様様様様様�
; �                 � � � �A� LIMIT   � �     �  TYPE   �                 �
; �   BASE 31..24   �G�0�0�V�         �P� DPL �         �   BASE 23..16   � 4
; �                 � � � �L�  19..16 � �     �0�1�0�B�1�                 �
; 把陳陳陳陳陳陳陳陳祖祖祖祖祖陳陳陳陳田祖陳陳祖祖祖祖祖祖陳陳陳陳陳陳陳陳�
; �                                   �                                   �
; �             BASE 15..0            �             LIMIT 15..0           � 0
; �                                   �                                   �
; 藩様様様様様様様様慷様様様様様様様様慷様様様様様様様様慷様様様様様様様様�


;Figure 7-3.  Task Register
;
;                          浜様様様様様様様様様様様様�
;                          �                         �
;                          �                         �
;                          �       TASK STATE        �
;                          �        SEGMENT          �^Q陳陳陳陳朕
;                          �                         �          �
;                          �                         �          �
;                          藩様様様様様様様様様様様様�          �
;           16-BIT VISIBLE             ^^                        �
;              REGISTER                �   HIDDEN REGISTER      �
;       浜様様様様様様様様様曜様様様様溶様様様様様僕様様様様様様詫様様融
;    TR �      SELECTOR      �      (BASE)        �       (LIMT)       �
;       藩様様様様冤様様様様擁様様様様様様様様様様瞥様様様様様様様様様夕
;                 �                    ^^                     ^^
;                 �                    青陳陳陳陳陳陳陳陳�   �
;                 �          GLOBAL DESCRIPTOR TABLE     �   �
;                 �        嬪様様様様様様様様様様様様�   �   �
;                 �        |     TSS DESCRIPTOR      |   �   �
;                 �        浜様様曜様様曜様様曜様様様�   �   �
;                 �        �      �     �     �      把陳�   �
;                 �        麺様様擁様様洋様様擁様様様�       �
;                 青陳陳陳^P�            �            把陳陳陳�
;                          藩様様様様様擁様様様様様様�
;                          |                         |
;                          塒様様様様様様様様様様様様�


;Figure 7-4.  Task Gate Descriptor
;
;   31                23               15                7             0
;  浜様様様様様様様様慷様様様様様様様謡用様様用様様様様謡様様様様様様様様融
;  艮臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼嘘 �     �         咳臼臼臼臼臼臼臼臼�
;  艮臼臼臼臼臼�(NOT USED)臼臼臼臼臼臼�P� DPL �0 0 1 0 1咳臼(NOT USED)臼臼� 4
;  艮臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼嘘 �     �         咳臼臼臼臼臼臼臼臼�
;  把陳陳陳陳陳陳陳陳陳陳陳陳陳陳陳陳津珍陳陳珍陳陳陳陳珍陳陳陳陳陳陳陳陳超
;  �                                  咳臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼�
;  �              SELECTOR            咳臼臼臼臼臼�(NOT USED)臼臼臼臼臼臼浦 0
;  �                                  咳臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼臼�
;  藩様様様様様様様様慷様様様様様様様謡様様様様様様様様謡様様様様様様様様夕

;Figure 7-5.  Task Gate Indirectly Identifies Task
;
;         LOCAL DESCRIPTOR TABLE              INTERRUPT DESCRIPTOR TABLE
;       嬪様様様様様様様様様様様様�           嬪様様様様様様様様様様様様�
;       |                         |           |                         |
;       |        TASK GATE        |           |        TASK GATE        |
;       浜様様用様様用様様用様様様�           浜様様用様様用様様用様様様�
;       �      �     �     �      �           �      �     �     �      �
;       把陳陳珍陳陳津陳陳珍陳陳陳�           把陳陳珍陳陳津陳陳珍陳陳陳�
;    敖超            �            �        敖超            �            �
;    �  藩様様様様様溶様様様様様様�        �  藩様様様様様溶様様様様様様�
;    �  |                         |        �  |                         |
;    �  |                         |        �  |                         |
;    �  塒様様様様様様様様様様様様�        �  塒様様様様様様様様様様様様�
;    青陳陳陳陳陳陳陳朕  敖陳陳陳陳陳陳陳陳�
;                     �  �    GLOBAL DESCRIPTOR TABLE
;                     �  �  嬪様様様様様様様様様様様様�
;                     �  �  |                         |
;                     �  �  |     TASK DESCRIPTOR     |
;                     �  �  浜様様用様様用様様用様様様�
;                     �  �  �      �     �     �      �
;                     �  青^P把陳陳珍陳陳津陳陳珍陳陳陳�
;                     青陳�^P�            �            把朕
;                           藩様様様様様溶様様様様様様�  �
;                           |                         |  �
;                           |                         |  �
;                           塒様様様様様様様様様様様様�  �
;                                                        �
;                           浜様様様様様様様様様様様様�  �
;                           �                         �  �
;                           �                         �  �
;                           �                         �  �
;                           �       TASK STATE        �  �
;                           �         SEGMENT         �  �
;                           �                         �  �
;                           �                         �  �
;                           �                         �  �
;                           藩様様様様様様様様様様様様�^Q潰
;

; 1.  A dynamic set that the processor updates with each switch from the
;      task. This set includes the fields that store:
;
;      �  The general registers (EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI).
;
;      �  The segment registers (ES, CS, SS, DS, FS, GS).
;
;      �  The flags register (EFLAGS).
;
;      �  The instruction pointer (EIP).
;
;      �  The selector of the TSS of the previously executing task (updated
;         only when a return is expected).
;
;  2.  A static set that the processor reads but does not change. This set
;      includes the fields that store:
;
;      �  The selector of the task's LDT.
;
;      �  The register (PDBR) that contains the base address of the task's
;         page directory (read only when paging is enabled).
;
;      �  Pointers to the stacks for privilege levels 0-2.
;
;      �  The T-bit (debug trap bit) which causes the processor to raise a
;         debug exception when a task switch occurs. (Refer to Chapter 12
;         for more information on debugging.)
;
;      �  The I/O map base (refer to Chapter 8 for more information on the
;         use of the I/O map).

;%include "video.inc"
;%include "gdt.inc"
;%include "util.inc"
;%include "keyboard.inc"
;%include "filesystem.inc"

;The multitasking support is a capability provided by the Intel architecture that the system designer can
;choose not to use. Many of the available multitasking operating systems that run on the Intel
;architecture do not use the hardware multitasking support provided. One of the key reasons is that the
;functionality of the hardware task switch can be completely supported using software task switching.
;Since many of the multitasking operating systems also require additional processing when switching
;tasks, performing the additional work is of little concern. In fact, the task switching may actually be
;faster because of the amount of processor information that must be preserved between task switches.
;For example, if an operating system elects not to use the local descriptor table, page unit or run with
;multiple privilege levels, the amount of information that must be preserved when switching tasks is
;less than what would be saved by using the hardware task-switching.


tss0:
DW 		0,0		; back link
DD		0		; esp0
DW		0,0		; ss0, RESERVED
DD		0		; esp1
DW		0,0		; ss1, RESERVED
DD		0		; esp2
DW		0,0		; ss2, RESERVED
DD		0		; cr3
tss0_eip:
DD		0,0x200		; eip, eflags
DD		0,0,0,0		; eax, ecx, edx, ebx
DD		0x0FFFF,0,0,0	; esp, ebp, esi, edi
DW		0x0010,0	; es, RESERVED
DW		0x0008,0	; cs, RESERVED
DW		0x0010,0	; ss, RESERVED
DW		0x0010,0	; ds, RESERVED
DW		0x0010,0	; fs, RESERVED
DW		0x0010,0	; gs, RESERVED
DW		0,0		; ldt, RESERVED
DW		0,0		; debug, IO perm. bitmap
tss0_size	equ 	$-tss0

tss1:
DW 		0,0		; back link
DD		0		; esp0
DW		0,0		; ss0, RESERVED
DD		0		; esp1
DW		0,0		; ss1, RESERVED
DD		0		; esp2
DW		0,0		; ss2, RESERVED
DD		0		; cr3
tss1_eip:
DD		0,0x200		; eip, eflags
DD		0,0,0,0		; eax, ecx, edx, ebx
DD		0x1FFFF,0,0,0	; esp, ebp, esi, edi
DW		0x0010,0	; es, RESERVED
DW		0x0008,0	; cs, RESERVED
DW		0x0010,0	; ss, RESERVED
DW		0x0010,0	; ds, RESERVED
DW		0x0010,0	; fs, RESERVED
DW		0x0010,0	; gs, RESERVED
DW		0,0		; ldt, RESERVED
DW		0,0		; debug, IO perm. bitmap
tss1_size	equ 	$-tss1

tss2:
DW 		0,0		; back link
DD		0		; esp0
DW		0,0		; ss0, RESERVED
DD		0		; esp1
DW		0,0		; ss1, RESERVED
DD		0		; esp2
DW		0,0		; ss2, RESERVED
DD		0		; cr3
tss2_eip:
DD		0,0x200		; eip, eflags
DD		0,0,0,0		; eax, ecx, edx, ebx
DD		0x2FFFF,0,0,0	; esp, ebp, esi, edi
DW		0x0010,0	; es, RESERVED
DW		0x0008,0	; cs, RESERVED
DW		0x0010,0	; ss, RESERVED
DW		0x0010,0	; ds, RESERVED
DW		0x0010,0	; fs, RESERVED
DW		0x0010,0	; gs, RESERVED
DW		0,0		; ldt, RESERVED
DW		0,0		; debug, IO perm. bitmap
tss2_size	equ 	$-tss2

tss3:
DW 		0,0		; back link
DD		0		; esp0
DW		0,0		; ss0, RESERVED
DD		0		; esp1
DW		0,0		; ss1, RESERVED
DD		0		; esp2
DW		0,0		; ss2, RESERVED
DD		0		; cr3
tss3_eip:
DD		0,0x200		; eip, eflags
DD		0,0,0,0		; eax, ecx, edx, ebx
DD		0x3FFFF,0,0,0	; esp, ebp, esi, edi
DW		0x0010,0	; es, RESERVED
DW		0x0008,0	; cs, RESERVED
DW		0x0010,0	; ss, RESERVED
DW		0x0010,0	; ds, RESERVED
DW		0x0010,0	; fs, RESERVED
DW		0x0010,0	; gs, RESERVED
DW		0,0		; ldt, RESERVED
DW		0,0		; debug, IO perm. bitmap
tss3_size	equ 	$-tss3

tss4:
DW 		0,0		; back link
DD		0		; esp0
DW		0,0		; ss0, RESERVED
DD		0		; esp1
DW		0,0		; ss1, RESERVED
DD		0		; esp2
DW		0,0		; ss2, RESERVED
DD		0		; cr3
tss4_eip:
DD		0,0x200		; eip, eflags
DD		0,0,0,0		; eax, ecx, edx, ebx
DD		0x4FFFF,0,0,0	; esp, ebp, esi, edi
DW		0x0010,0	; es, RESERVED
DW		0x0008,0	; cs, RESERVED
DW		0x0010,0	; ss, RESERVED
DW		0x0010,0	; ds, RESERVED
DW		0x0010,0	; fs, RESERVED
DW		0x0010,0	; gs, RESERVED
DW		0,0		; ldt, RESERVED
DW		0,0		; debug, IO perm. bitmap
tss4_size	equ 	$-tss4

tss_number	DB	0
tss_address:
		DD	tss1		
		DD	tss2
		DD	tss3
		DD	tss4
tss_address_size:

init_tss:
mov edi, tss0_eip
mov dword [es:edi], sheduler 
mov edi, tss1_eip
mov dword [es:edi], task1 
mov edi, tss2_eip
mov dword [es:edi], task2
mov edi, tss3_eip
mov dword [es:edi], task3
mov edi, tss4_eip
mov dword [es:edi], task4

mov eax, tss0
mov edi, sys_tss
mov word [es:edi + 2],ax
shr eax,16
mov byte [es:edi + 4],al
mov byte [es:edi + 7],ah
mov eax, tss1
mov edi, user_tss1
mov word [es:edi + 2],ax
shr eax,16
mov byte [es:edi + 4],al
mov byte [es:edi + 7],ah
retf

;====================================================================;
; Sheduler Task                                                      ;
;====================================================================;
sheduler:
xor eax, eax
mov al, [tss_number]
mov ebx, 4
mul ebx
mov esi, tss_address
add esi, eax
cmp esi, tss_address_size
jne inc_task1
mov esi, tss_address
mov byte [tss_number], 0
inc_task1:
inc byte [tss_number]
mov eax, dword [es:esi]
mov edi, user_tss1
mov word [es:edi + 2],ax
shr eax,16
mov byte [es:edi + 4],al
mov byte [es:edi + 5], 10001001b
mov byte [es:edi + 7],ah
;;; Show task number on screen
VIDEO_TEXTMODE_write_mensage            0x01,0x01,0x08,tasks_string
mov edi, 0x000B8000 + 160*0x01 + 2*7
mov eax, 4
xor ebx, ebx
mov bl, byte [tss_number] 
mul ebx
add edi, eax
mov al, 48
add al, byte [tss_number]
mov ah, 14
mov word [es:edi], ax
;;; JUMP
jmp user_tss1-gdt:0	; jump to task, this will enable interrupts
;;; For DELAYS with PIT
;;; add dword [TIMER_PIT_TICKS], 65536
mov al, 0x20
out 0x20, al
jmp dword 0x0008:sheduler


;========================================================================;
;                                  TASKS                                 ;
;========================================================================;
task1:
;;; SHELL
FILESYSTEM_goto_root
VIDEO_TEXTMODE_set_line		     0x03
next_command:
clean_str	command_string 
mov eax, command_string
mov [command_position], eax
VIDEO_TEXTMODE_print_line            0x01,0x03,shell_string
mov edi, 0x000B8000 + 2*21

mov eax, 160
xor ebx, ebx
mov bl, [VIDEO_TEXTMODE_LINE]
mul ebx 
add edi, eax

mov dword [command_input_position], edi
mov byte [color], 0x07
next_ascii:
mov al, 95
mov ah, 135
mov word [es:edi], ax
KEYBOARD_get_key      	key 
cmp edi, dword [command_input_position] 
je skip_backspace1
cmp byte [key], KEYBOARD_BACKSPACE_KEY 
jne skip_backspace1
mov ax, 0
mov word [es:edi], ax
sub edi, 2
dec dword [command_position]
skip_backspace1:
mov al, [key] 
mov [ascii], al
KEYBOARD_map		ascii
mov al, [ascii]
mov ah, [color]

mov esi, [command_position]
mov byte [es:esi], al

mov word [es:edi], ax
cmp byte [key], KEYBOARD_BACKSPACE_KEY 
je skip_backspace2
add edi, 2
inc dword [command_position]
skip_backspace2:
cmp byte [key], KEYBOARD_ENTER_KEY 
jne near next_ascii


%include "shellcmds.inc"

task2:
;;; TODO: buffer of 80 chars, SPACES when end of string 
repeat_scroll:

;;;;;;;;;; RIGHT SCROLL
mov edi, 0x000B8000 + 160*0x03
mov esi, scroll_mensage
mov ah, 0x0E	; COLOR
pusha
next_s:
popa
times 2 dec edi
pusha
next_c:
mov al, byte [es:esi]
mov word [es:edi], ax
inc esi
times 2 inc edi
cmp edi, 0x000B8000 + 160*0x03
jne next_c
;;; TIMER_pit_msdelay        100 
TIMER_rtc_msdelay        100 
cmp byte [es:esi], 0
jne next_s
;;;;;;;;;; CENTER SCROLL
popa
;mov esi, scroll_mensage
;mov ah, 0x0E	; COLOR
pusha
next_s1:
popa 
times 2 dec edi
cmp edi, 0x000B8000 + 160*0x02 - 2*0x01
je skip_next_s1
pusha
next_c1:
mov al, byte [es:esi]
mov word [es:edi], ax
inc esi
times 2 inc edi
cmp byte [es:esi], 0
jne next_c1
mov word [es:edi], 0
;;; TIMER_pit_msdelay        100 
TIMER_rtc_msdelay        100 
jmp dword 0x0008:next_s1
skip_next_s1:
;;;;;;;;;; LEFT SCROLL
mov edi, 0x000B8000 + 160*0x02
mov esi, scroll_mensage
mov ah, 0x0E	; COLOR
pusha
next_s2:
popa
inc esi
pusha
next_c2:
mov al, byte [es:esi]
mov word [es:edi], ax
inc esi
times 2 inc edi
cmp byte [es:esi], 0
jne next_c2
mov word [es:edi], 0
;;; TIMER_pit_msdelay        100 
TIMER_rtc_msdelay        100 
cmp edi, 0x000B8000 + 160*0x02 + 2*0x01 
jne next_s2
times 2 dec edi
mov word [es:edi], 0

cmp byte [faded], 1
je skip_faded
call dword 0x0008:VIDEO_VGA_fadein
;;; FADE IN, after all task's had shown their program image
skip_faded:
jmp dword 0x0008:repeat_scroll
retf

task3:
;;; TIME
repeat_time:
CMOS_rtc_ready
CMOS_read_address               CMOS_TIME_HOUR_ADDRESS,dword [dd_number]
number_to_text  [dd_number], 16, time_string+1 
CMOS_rtc_ready
CMOS_read_address               CMOS_TIME_MINUTE_ADDRESS,dword [dd_number]
number_to_text  [dd_number], 16, time_string+4
CMOS_rtc_ready
CMOS_read_address               CMOS_TIME_SECOND_ADDRESS,dword [dd_number]
number_to_text  [dd_number], 16, time_string+7
VIDEO_TEXTMODE_write_mensage            0x01,35,0x0F,time_string
mov ax, '00'
mov edi, time_string
mov word [es:edi], ax
mov word [es:edi+3], ax
mov word [es:edi+6], ax
jmp dword 0x0008:repeat_time
retf

task4:
;;; DATE
repeat_date:
CMOS_rtc_ready
CMOS_read_address       	CMOS_DATE_DAYOFMONTH_ADDRESS,dword [dd_number]
number_to_text  [dd_number], 16, date_string+1 
CMOS_rtc_ready
CMOS_read_address       	CMOS_DATE_MONTH_ADDRESS,dword [dd_number]
number_to_text  [dd_number], 16, date_string+4
CMOS_rtc_ready
CMOS_read_address               CMOS_DATE_YEAR_ADDRESS,dword [dd_number]
number_to_text  [dd_number], 16, date_string+7
VIDEO_TEXTMODE_write_mensage		0x01, 65, 0x0F, date_string
mov ax, '00'
mov edi, date_string
mov word [es:edi], ax
mov word [es:edi+3], ax
mov word [es:edi+6], ax
jmp dword 0x0008:repeat_date
retf


;=====================================================================
;shell_string		DB '[root@localhost root]$',0
shell_string		DB '[root@localhost /]$',0
COMMAND_TEMP_VALUE	DD 0
COMMAND_DIR_STRING   	DB 'dir',0
COMMAND_CPU_STRING   	DB 'cpu',0
COMMAND_RAM_STRING   	DB 'ram',0
COMMAND_PCI_STRING   	DB 'pci',0
COMMAND_BIOSPNP_STRING 	DB 'biospnp',0
COMMAND_VESA_STRING   	DB 'vesa',0
COMMAND_ISAPNP_STRING 	DB 'isapnp',0
COMMAND_REBOOT_STRING 	DB 'reboot',0
COMMAND_CODERS_STRING 	DB 'coders',0
COMMAND_HELP_STRING 	DB 'help',0
line_string             DB '                                                   rev                         ',0
command_input_position	DD 0
key			DB 0
ascii			DB 0
color			DB 0
scroll_mensage  	DB 'Welcome to Heaven Operating System',0
scroll_mensage_size 	equ $-scroll_mensage
time_string		DB '00:00:00',0
date_string		DB '00-00-00',0
tasks_string		DB 'Tasks : 1 2 3 4',0
dd_number		DD 0
command_string		DB '                                            ',0
command_position	DD 0
command_error_msg	DB 'unknown command',0
file_msg		DB '                                                                               ',0
FILE_ENTRY		DD 0
faded			DB 0
reboot_msg		DB 'Rebooting in 5 seconds',0
reboot_time		DB '.',0
cpu_sign_msg            DB 'Checking CPU Signature...',0
cpu_type_msg            DB 'Checking CPU Family...',0
cpu_type                DB '80 86',0
cpu_id_msg              DB 'Checking CPU ID...',0
pc_ram_msg              DB 'Checking RAM...',0
pc_ram                  DB '     MB',0
pci_devices_msg         DB 'Checking PCI Devices...',0
device                  DW 0
device_id               DD 0
class_id                DB 0
int_line                DD 0
isapnp_devices_msg      DB 'Checking ISA PNP Devices...',0
pnp_bios_msg            DB 'Checking PNP BIOS...',0
vesa_card_msg           DB 'Checking VESA Device...',0
found_msg          	DB 'Found !',0
notfound_msg       	DB 'Not Found !',0

help_msg0		DB 'reboot - Reboot PC',0
help_msg1		DB 'cpu - Check CPU',0
help_msg2		DB 'ram - Check RAM',0
help_msg3		DB 'pci - Check PCI Devices',0
help_msg4		DB 'isapnp - Check ISA PNP Devices',0
help_msg5		DB 'biospnp - Check PNP BIOS Devices',0
help_msg6		DB 'vesa - Check VESA Device',0
help_msg7		DB 'coders - List of Coders',0


