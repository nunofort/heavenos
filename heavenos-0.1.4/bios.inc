
; BIOS Cold Reboot
BIOS_cold_boot:
mov edi, 16 * BIOS_DATA_SEGMENT + BIOS_POST_RESET_FLAG_OFFSET
mov word [es:edi], BIOS_COLD_REBOOT_FLAG
jmp dword 0x0008: 16 * 0xFFFF + 0x0000

; BIOS Warm Reboot
BIOS_warm_boot:
mov edi, 16 * BIOS_DATA_SEGMENT + BIOS_POST_RESET_FLAG_OFFSET
mov word [es:edi], BIOS_WARM_REBOOT_FLAG
jmp dword 0x0008: 16 * 0xFFFF + 0x0000

; get BIOS Timer Ticks since midnight [0040:006C]
; updated every 55 mseconds(execute INT 0x08 - timer)
; PM Address(REAL) = 16 * RM Segment + RM Offset
%macro BIOS_read_clock_ticks		1 
mov esi, 16 * BIOS_DATA_SEGMENT + BIOS_CLOCK_TICKS_OFFSET
mov eax, dword [es:esi]
mov %1, eax
%endmacro

%macro BIOS_write_clock_ticks		1
mov edi, 16 * BIOS_DATA_SEGMENT + BIOS_CLOCK_TICKS_OFFSET
mov eax, %1
mov dword [es:edi], eax
%endmacro

%macro BIOS_inc_clock_ticks	0
mov edi, 16 * BIOS_DATA_SEGMENT + BIOS_CLOCK_TICKS_OFFSET
inc dword [es:edi]
%endmacro

%macro BIOS_dec_clock_ticks		0
mov edi, 16 * BIOS_DATA_SEGMENT + BIOS_CLOCK_TICKS_OFFSET
dec dword [es:edi]
%endmacro

;check PNP_BIOS Structure String (0x000F0000-0x000FFFFF)
%macro BIOS_pnp_signature	1
mov edi, 0x000F0000
mov esi, 0x000FFFFF
%%check_pnp:
mov eax, dword [es:edi]
inc edi
cmp eax, [BIOS_PNP_STRING]
je %%end_check
cmp edi, [esi-3]
jne %%check_pnp
mov edi, 1
%%end_check:
dec edi
mov %1, eax 
%endmacro

;=========================================================================;
; BIOS                                                                    ;
;=========================================================================;
BIOS_PNP_STRING         	DD '$PnP'
BIOS_DATA_SEGMENT		equ 0x0040
BIOS_CLOCK_TICKS_OFFSET		equ 0x006C
BIOS_POST_RESET_FLAG_OFFSET	equ 0x0072
BIOS_COLD_REBOOT_FLAG		equ 0x0000
BIOS_WARM_REBOOT_FLAG		equ 0x1234

;
;Offset    Length    Value    Description
;0h        4 BYTES    $PnP    Signature    Generic
;               (ASCII)
;04h    BYTE    Varies    Structure Revision    01h
;05h    BYTE    Varies     Length (in 16 byte increments)    Generic
;06h    WORD    Varies    Offset of next Header (0000h if none)    Generic
;08h    BYTE    00h       Reserved    Generic
;09h    BYTE    Varies    Checksum    Generic
;0Ah    DWORD    Varies    Device Identifier    PnP Specific
;0Eh    WORD    Varies    Pointer to Manufacturer String (Optional)    PnP Specific
;10h    WORD    Varies    Pointer to Product Name String (Optional)    PnP Specific
;12h    3 BYTE    Varies    Device Type Code    PnP Specific
;15h    BYTE    Varies    Device Indicators    PnP Specific
;16h    WORD    Varies    Boot Connection Vector - Real/Protected mode    PnP Specific
;                         (0000h if none)
;18h    WORD    Varies    Disconnect Vector - Real/Protected mode (0000h if    PnP Specific
;                         none)
;1Ah    WORD    Varies    Bootstrap Entry Point - Real/Protected mode  (0000h    PnP Specific
;                         if none)
;1Ch    WORD    0000h    Reserved    PnP Specific
;1Eh    WORD    Varies    Static Resource Information Vector- Real/Protected    PnP Specific
;                         mode (0000h if none)


