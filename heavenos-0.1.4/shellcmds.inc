
mov esi, [command_position]
mov byte [es:esi], 0
;VIDEO_TEXTMODE_write_mensage            0x03, 0x01, 0x0F, command_string
; "help"
cmp_str command_string, COMMAND_HELP_STRING, [COMMAND_TEMP_VALUE]
cmp dword [COMMAND_TEMP_VALUE], 1
jne skip_help
call dword 0x0008:help_info
jmp near next_command
skip_help:
; "coders"
cmp_str command_string, COMMAND_CODERS_STRING, [COMMAND_TEMP_VALUE]
cmp dword [COMMAND_TEMP_VALUE], 1
jne skip_coders
call dword 0x0008:coders_info
jmp near next_command
skip_coders:
; "quit"
cmp_str command_string, COMMAND_REBOOT_STRING, [COMMAND_TEMP_VALUE]
cmp dword [COMMAND_TEMP_VALUE], 1
jne skip_reboot
call dword 0x0008:reboot
jmp near next_command
skip_reboot:
; "dir"
;cmp_str command_string, COMMAND_DIR_STRING, [COMMAND_TEMP_VALUE]
;cmp dword [COMMAND_TEMP_VALUE], 1
;jne skip_dir_info
;call dword 0x0008:dir_info
;jmp near next_command
skip_dir_info:
; "cd <directory>"
; "cpu"
cmp_str command_string, COMMAND_CPU_STRING, [COMMAND_TEMP_VALUE]
cmp dword [COMMAND_TEMP_VALUE], 1
jne skip_cpu_info
call dword 0x0008:cpu_info
jmp near next_command
skip_cpu_info:
; "ram"
cmp_str command_string, COMMAND_RAM_STRING, [COMMAND_TEMP_VALUE]
cmp dword [COMMAND_TEMP_VALUE], 1
jne skip_ram_info
call dword 0x0008:ram_info
jmp near next_command
skip_ram_info:
; "pci"
cmp_str command_string, COMMAND_PCI_STRING, [COMMAND_TEMP_VALUE]
cmp dword [COMMAND_TEMP_VALUE], 1
jne skip_pci_info
call dword 0x0008:pci_info
jmp near next_command
skip_pci_info:
; "isapnp"
cmp_str command_string, COMMAND_ISAPNP_STRING, [COMMAND_TEMP_VALUE]
cmp dword [COMMAND_TEMP_VALUE], 1
jne skip_isapnp_info
call dword 0x0008:isapnp_info
jmp near next_command
skip_isapnp_info:
; "biospnp"
cmp_str command_string, COMMAND_BIOSPNP_STRING, [COMMAND_TEMP_VALUE]
cmp dword [COMMAND_TEMP_VALUE], 1
jne skip_biospnp_info
call dword 0x0008:biospnp_info
jmp near next_command
skip_biospnp_info:
; "vesa"
cmp_str command_string, COMMAND_VESA_STRING, [COMMAND_TEMP_VALUE]
cmp dword [COMMAND_TEMP_VALUE], 1
jne skip_vesa_info
call dword 0x0008:vesa_info
jmp near next_command
skip_vesa_info:
; "UNKNOWN COMMAND"
VIDEO_TEXTMODE_print_line		0x01,0x07, command_error_msg
jmp near next_command


;=========================================================================;
;                             SHELL COMMANDS                              ;
;=========================================================================;
reboot:
VIDEO_TEXTMODE_print_line            0x01,0x07,reboot_msg
xor eax, eax
mov al, [VIDEO_TEXTMODE_LINE]
mov [dword_number], eax
VIDEO_TEXTMODE_write_mensage		[dword_number], 23, 0x07, reboot_time
TIMER_rtc_msdelay        1000
VIDEO_TEXTMODE_write_mensage		[dword_number], 24, 0x07, reboot_time
TIMER_rtc_msdelay        1000
VIDEO_TEXTMODE_write_mensage		[dword_number], 25, 0x07, reboot_time
TIMER_rtc_msdelay        1000
VIDEO_TEXTMODE_write_mensage		[dword_number], 26, 0x07, reboot_time
TIMER_rtc_msdelay        1000
VIDEO_TEXTMODE_write_mensage		[dword_number], 27, 0x07, reboot_time
TIMER_rtc_msdelay        1000

;KEYBOARD_check_key      KEYBOARD_ENTER_KEY
mov edx, $              ; EDX = EIP , for debugging
jmp dword 0x0008:KEYBOARD_system_reset
;jmp dword 0x0008:BIOS_cold_boot

help_info:
;=================================================;
; Help Information                                ;
;=================================================;
VIDEO_TEXTMODE_print_line            0x01,0x07,help_msg0
VIDEO_TEXTMODE_print_line            0x01,0x07,help_msg1
VIDEO_TEXTMODE_print_line            0x01,0x07,help_msg2
VIDEO_TEXTMODE_print_line            0x01,0x07,help_msg3
VIDEO_TEXTMODE_print_line            0x01,0x07,help_msg4
VIDEO_TEXTMODE_print_line            0x01,0x07,help_msg5
VIDEO_TEXTMODE_print_line            0x01,0x07,help_msg6
VIDEO_TEXTMODE_print_line            0x01,0x07,help_msg7
retf

coders_info:
;=================================================;
; Team Information                                ;
;=================================================;
;VIDEO_TEXTMODE_print_line            0x01,0x07,coders_msg1
;inc byte [VIDEO_TEXTMODE_LINE]
retf

;dir_info:
;=================================================;
; Directory List                                  ;
;=================================================;
;mov dword [FILE_ENTRY], 0
;next_file:
;mov eax, [FILE_ENTRY]
;mov dword [FILESYSTEM_TEMP_VALUE], eax
;FILESYSTEM_read_dir_entry       [FILESYSTEM_TEMP_VALUE]
;cmp dword [FILESYSTEM_TEMP_VALUE], 0xFFFFFFFF
;je near skip_entry
;;number_to_text  [FILESYSTEM_TEMP_VALUE], 10, file_msg+10 
;;VIDEO_TEXTMODE_print_line            0x01,0x07,file_msg
;FILESYSTEM_read_file_header     [FILESYSTEM_TEMP_VALUE]
;;number_to_text  [FILESYSTEM_TEMP_VALUE], 10, file_msg+30
;;VIDEO_TEXTMODE_print_line            0x01,0x07,file_msg

;clean_str  	file_msg
;mov esi, FILE_HEADER
;xor eax, eax
;mov al, byte [es:esi]
;mov dword [FILESYSTEM_TEMP_VALUE], eax
;FILESYSTEM_attrib  [FILESYSTEM_TEMP_VALUE], file_msg+10
;mov esi, FILE_HEADER+36
;xor eax, eax
;mov ax, word [es:esi]
;mov dword [FILESYSTEM_TEMP_VALUE], eax
;number_to_text  [FILESYSTEM_TEMP_VALUE], 10, file_msg+22
;copy_str	FILE_HEADER+1, file_msg+24 
;VIDEO_TEXTMODE_print_line            0x01,0x07,file_msg
;skip_entry:
;inc dword [FILE_ENTRY]
;cmp dword [FILE_ENTRY], 10 
;jl near next_file
;retf

cpu_info:
;================================================;
; Checking CPU Family...                         ;
;================================================;
VIDEO_TEXTMODE_print_line            0x01,0x07,cpu_type_msg
mov dword [dword_number], 0
CPU_family      dword_number
number_to_text  [dword_number], 10, cpu_type+2
xor eax, eax
mov al, [VIDEO_TEXTMODE_LINE]
mov [dd_number], eax
VIDEO_TEXTMODE_write_mensage            [dd_number],0x18,0x07,cpu_type

;CPU_check_cpuid        [dword_number]
;cmp dword [dword_number], 0
;je skip_cpu

cmp dword [dword_number], 5
jge detect_cpu
jmp dword 0x0008:skip_cpu
detect_cpu:
;===============================================;
; Checking CPU Signature...                     ;
;===============================================;
VIDEO_TEXTMODE_print_line            0x01,0x07,cpu_sign_msg
call dword 0x0008:CPU_signature
xor eax, eax
mov al, [VIDEO_TEXTMODE_LINE]
mov [dd_number], eax
VIDEO_TEXTMODE_write_mensage         [dd_number],0x1B,0x07,CPU_SIGNATURE
;===============================================;
; Checking CPU ID...                            ;
;===============================================;
VIDEO_TEXTMODE_print_line            0x01,0x07,cpu_id_msg
call dword 0x0008:CPU_id
CPU_detect      dword_number2
number_to_text  [CPU_ID], 2, line_string+15
copy_str        [CPU_STRING], line_string+18
number_to_text  [dword_number2], 10, line_string+55
VIDEO_TEXTMODE_print_line            0x01,0x07, line_string
clean_str       line_string
;number_to_text [CPU_FLAGS], 2, line_string+32
;VIDEO_TEXTMODE_print_line           0x01,0x07, line_string
;clean_str      line_string
skip_cpu:
retf

ram_info:
;===============================================;
; Checking RAM...                               ;
;===============================================;
VIDEO_TEXTMODE_print_line            0x01,0x07,pc_ram_msg
MEMORY_get_ram          [dword_number]
number_to_text  [dword_number], 10, pc_ram+4
xor eax, eax
mov al, [VIDEO_TEXTMODE_LINE]
mov [dd_number], eax
VIDEO_TEXTMODE_write_mensage            [dd_number],0x11,0x07,pc_ram
retf

pci_info:
;===============================================;
; Checking PCI Devices...                       ;
;===============================================;
VIDEO_TEXTMODE_print_line            0x01,0x07,pci_devices_msg
PCI_get_type            [pci_type]

mov word [device], 0
cmp dword [pci_type], 0
jne next_pci_device
xor eax, eax
mov al, [VIDEO_TEXTMODE_LINE]
mov [dd_number], eax
VIDEO_TEXTMODE_write_mensage         [dd_number],0x19,0x07,notfound_msg
jmp dword 0x0008:end_pci
next_pci_device:
PCI_get_device_info     [pci_type],[device],pci_device
mov eax, dword [es:pci_device]
cmp eax, 0xFFFFFFFF
jne show_device
inc word [device]
jmp dword 0x0008:next_pci_device
show_device:
mov [device_id], eax
mov al, byte [es:pci_device+0x0B]
mov [class_id], al
mov al, byte [es:pci_device+0x3C]
mov byte [int_line], al
PCI_detect_device       [device_id], [class_id], [pci_device_string], [pci_class_string]
number_to_text  [device_id], 16, line_string+9
number_to_text  [int_line], 10, line_string+13
copy_str        [pci_device_string], line_string+15
copy_str        [pci_class_string], line_string+50
VIDEO_TEXTMODE_print_line            0x01,0x07,line_string
clean_str       line_string
inc word [device]
cmp word [device], 512
jg end_pci
jmp dword 0x0008:next_pci_device
end_pci:
retf

isapnp_info:
;===============================================;
; Checking ISA PNP Devices...                   ;
;===============================================;
VIDEO_TEXTMODE_print_line            0x01,0x07,isapnp_devices_msg
mov ax, ISAPNP_DEVICE_PORT_START+3
mov [ISAPNP_DEVICE_PORT], ax
call dword 0x0008:ISAPNP_init_isolation

next_port:
call dword 0x0008:ISAPNP_isolation
mov esi, ISAPNP_ENCODED_SIGNATURE
mov eax, dword [ds:esi]
mov ebx, dword [ds:esi+4]
cmp eax, dword 'FFFF'
jne show_isapnp
cmp ebx, dword 'FFFF'
jne show_isapnp
jmp dword 0x0008:check_port

show_isapnp:
call dword 0x0008:ISAPNP_detect
copy_str        ISAPNP_ENCODED_SIGNATURE, line_string+2
xor eax, eax
mov ax, [ISAPNP_DEVICE_PORT]
mov [dword_number], eax
number_to_text  [dword_number], 16, line_string+13
copy_str        [ISAPNP_CARD_STRING], line_string+15
VIDEO_TEXTMODE_print_line            0x01,0x07,line_string
clean_str       line_string
dec word [ISAPNP_DEVICE_PORT]

check_port:
inc word [ISAPNP_DEVICE_PORT]
mov ax, [ISAPNP_DEVICE_PORT]
;;;cmp ax, ISAPNP_DEVICE_PORT_END
cmp ax, 0x204
je end_isapnp
jmp dword 0x0008:next_port
end_isapnp:
retf

biospnp_info:
;===============================================;
; Checking PNP BIOS...                          ;
;===============================================;
VIDEO_TEXTMODE_print_line            0x01,0x07,pnp_bios_msg
BIOS_pnp_signature      [dword_number]
cmp dword [dword_number], '$PnP'
jne not_found
xor eax, eax
mov al, [VIDEO_TEXTMODE_LINE]
mov [dd_number], eax
VIDEO_TEXTMODE_write_mensage         [dd_number],0x16,0x07,found_msg
jmp short skip_biospnp
not_found:
xor eax, eax
mov al, [VIDEO_TEXTMODE_LINE]
mov [dd_number], eax
VIDEO_TEXTMODE_write_mensage         [dd_number],0x16,0x07,notfound_msg
skip_biospnp:
retf

vesa_info:
;===============================================;
; Checking VESA Graphics Card...                ;
;===============================================;
VIDEO_TEXTMODE_print_line            0x01,0x07,vesa_card_msg
VESA_signature      [dword_number]
cmp dword [dword_number], 0
je not_found1
xor eax, eax
mov al, [VIDEO_TEXTMODE_LINE]
mov [dd_number], eax
VIDEO_TEXTMODE_write_mensage         [dd_number],0x19,0x07,found_msg
jmp short skip_vesa
not_found1:
xor eax, eax
mov al, [VIDEO_TEXTMODE_LINE]
mov [dd_number], eax
VIDEO_TEXTMODE_write_mensage         [dd_number],0x19,0x07,notfound_msg
skip_vesa:
retf



