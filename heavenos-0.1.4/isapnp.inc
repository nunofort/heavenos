;==============================================================================
; FUNCTION     WRITE ADDRESS                 WRITE DATA
;==============================================================================
; 0x00         Set Read Port                 Port Address(PORT>>2)
; 0x01         SERIAL ISOLATION
; 0x02         CONFIG CONTROL                1 RESET
;                                            2 WAIT_FOR_KEY(normal state after power on)
;                                            4 RESET_CSN(all)
;                                            7 RESET_DRV(combination of above)
; 0x03         WAKEUP                        > logical card number(0 = reset all cards)
; 0x04         RESOURCE DATA                 < read (Read Port Address)
; 0x05         STATUS                        < read (Read Port Address)
; 0x06         SET CARD NUMBER               > logical card number
; 0x07         SET DEVICE NUMBER             > logical device number
; 0x30         ACTIVE ADDRESS                0x01 on reads
; 0x31         IO RANGECHECK
; 0x60         PORT
; 0x70         IRQ
; 0x74         DMA
;=============================================================================
; READ ADDRESS = 0x200<->0x3FF
; 0203 020b 0213 021b 0223 022b 0233 023b 0243 024b 0253 025b 0263 026b 0273
; 027b 0283 028b 0293 029b 02a3 02ab 02b3 02bb 02c3 02cb 02d3 02db 02e3 02eb
; 02f3 02fb 0303 030b 0313 031b 0323 032b 0333 033b 0343 034b 0353 035b 0363
; 036b 0373 037b 0383 038b 0393 039b 03a3 03ab 03b3 03bb 03c3 03cb 03d3 03db
; 03e3 03eb 03f3 03fb
;
; CSN = CARD SELECT NUMBER
; SERIAL NUMBER(72-bit) = 4-Byte Encoded ID + 4-Byte Serial Number + 1-Byte Checksum
; Encoded ID = 2-Byte Encoded Model Name + 2-Byte Product Number
; =>Encoded Model Name  - Bit 15 is not used
; 01234 56789 01234
; xxxxx:xxxxx:xxxxx
; 1:1:1 = AAA
; 26:26:26 = ZZZ
; =>Product Number
; 0x1234 = Product 0x123 Rev 0x4
; Bit 1 = the card sends 0x55 and 0xAA on the READ ADDRESS
; Bit 0 = the card sends nothing, but listens instead
; ### CARD ###
; 9 BYTE SERIAL NUMBER
; pnp version
; card name
; vendor id
;   device id 0
;     device name
;     resource config 0
;     resource config n
;     resource end
;   device id n
;     device name
;     resource configs 0 to n
;     resource end
; end
; Each Record has <TAG><OPTIONAL LENGTH><DATA>
; SHORT FORMAT: 0TTTTLLL data
; TTTT: 4-bit field, 16 possible types
; -----------------------------------------------
; 0x1 = Plug and Play version number
; 0x2 = Logical Device ID
; 0x3 = Compatible Device ID
; 0x4 = IRQ Format
; 0x5 = DMA Format
; 0x6 = Start Dependent Function
; 0x7 = End Dependent Function
; 0x8 = I/O Port Descriptor
; 0x9 = Fixed Location I/O Port Descriptor
; 0xA-0xD = RESERVED
; 0xE = Vendor Defined
; 0xF = End Tag
; -----------------------------------------------
; LLL: 3-bit field, length of data
;  LONG FORMAT: 1TTTTTTT LL data
; TTTTTTT: 7-bit field, 128 possible types
; -----------------------------------------------
; 0x1 = Memory Range Descriptor
; 0x2 = Identifier String(ANSI)
; 0x3 = Identifier String(Unicode)
; 0x4 = Vendor Defined
; 0x5 = 32-Bit Memory Range Descriptor
; 0x6 = 32-Bit Fixed Location Memory Range Descriptor
; 0x7-0x7F = RESERVED
; -----------------------------------------------
; LL: 2-byte length of data
;============================================================================;

;%include "idt.inc"

%macro ISAPNP_write_address	1
mov al, %1
mov dx, ISAPNP_WRITE_ADDRESS_PORT
out dx, al 
jmp short $+2		; Delay at least 10useconds
;TIMER_delay	MSEC_PIT_FREQUENCY, 1
%endmacro

%macro ISAPNP_write_data	1
mov al, %1
mov dx, ISAPNP_WRITE_DATA_PORT
out dx, al
%endmacro

%macro ISAPNP_read_data		1
mov dx, [ISAPNP_DEVICE_PORT]
in al, dx
mov %1, al
%endmacro

%macro ISAPNP_write_byte	2
ISAPNP_write_address	%1
ISAPNP_write_data	%2
%endmacro

%macro ISAPNP_reset		0
ISAPNP_write_byte	0x02,0x01
%endmacro

%macro ISAPNP_reset_csn		0
ISAPNP_write_byte	0x02,0x04
%endmacro

%macro ISAPNP_reset_drv		0
ISAPNP_write_byte	0x02,0x07
;jmp short $+2		; Delay at least 2mseconds
;TIMER_delay	MSEC_PIT_FREQUENCY, 2 
TIMER_rtc_msdelay	2
%endmacro

%macro ISAPNP_wait_state	0
ISAPNP_write_byte	0x02,0x02
;jmp short $+2		; Delay at least 1msecond
;TIMER_delay	MSEC_PIT_FREQUENCY, 1
TIMER_rtc_msdelay	1
%endmacro

%macro ISAPNP_wakeup_csn	1
ISAPNP_write_byte	0x03, %1
%endmacro

%macro ISAPNP_set_card		1
ISAPNP_write_byte	0x06, %1
%endmacro

%macro ISAPNP_set_device	1
ISAPNP_write_byte	0x07, %1
%endmacro

%macro ISAPNP_device_activate		1
ISAPNP_set_device	%1
ISAPNP_write_byte	0x02,0x01
jmp short $+2		; Delay at least 250useconds
;TIMER_delay	MSEC_PIT_FREQUENCY, 1
%endmacro

%macro ISAPNP_device_deactivate	1
ISAPNP_set_device	%1
ISAPNP_write_byte	0x02,0x00
jmp short $+2		; Delay at least 250useconds
;TIMER_delay	MSEC_PIT_FREQUENCY, 1
%endmacro

%macro ISAPNP_serial_isolation	0
ISAPNP_write_address	0x01
%endmacro

%macro ISAPNP_set_device_port	0
mov bx, [ISAPNP_DEVICE_PORT]
shr bx, 2
ISAPNP_write_byte	0x00, bl
;jmp short $+2		; Delay at least 1000useconds
;TIMER_delay	MSEC_PIT_FREQUENCY, 1 
TIMER_rtc_msdelay	1
%endmacro

%macro ISAPNP_get_resource_data	1
ISAPNP_write_address	0x04
ISAPNP_read_data	%1
%endmacro

%macro ISAPNP_get_status	1
ISAPNP_write_address	0x05
ISAPNP_read_data	%1
%endmacro

%macro ISAPNP_activate_address	0
ISAPNP_write_address	0x30
%endmacro

%macro ISAPNP_check_range	0
ISAPNP_write_address	0x31
%endmacro

ISAPNP_initkey:
; Delay at least 1msecond 
;TIMER_delay	MSEC_PIT_FREQUENCY, 1
TIMER_rtc_msdelay	1
ISAPNP_write_address	0
ISAPNP_write_address	0
mov esi, ISAPNP_INITKEY
next_key:
mov al, byte [ds:esi]
mov byte [ISAPNP_TEMP_DATA], al
ISAPNP_write_address	[ISAPNP_TEMP_DATA]	
inc esi
cmp esi, ISAPNP_INITKEY+32 
jne next_key
;jmp dword 0x0008:next_key
;end_initkey:
retf

ISAPNP_init_isolation:
ISAPNP_wait_state
call dword 0x0008:ISAPNP_initkey
ISAPNP_reset_drv
ISAPNP_wait_state
call dword 0x0008:ISAPNP_initkey
ISAPNP_wakeup_csn 0x00
ISAPNP_set_device_port
ISAPNP_serial_isolation
; Delay at least 1msecond
;TIMER_delay	MSEC_PIT_FREQUENCY, 1
TIMER_rtc_msdelay	1
retf

ISAPNP_isolation:		
mov edi, ISAPNP_ENCODED_SIGNATURE
mov dword eax, 'FFFF'
mov dword ebx, 'FFFF'
mov dword [ds:edi], eax
mov dword [ds:edi+4], ebx
mov edi, ISAPNP_SERIAL_NUMBER
mov ax, [ISAPNP_DEVICE_PORT]
mov word [ds:edi], ax
add edi, 2
mov dword [ISAPNP_TEMP_ADDRESS], edi
mov dword [ISAPNP_TEMP_VALUE], 0
mov dword [ISAPNP_TEMP_BITMASK], 1
mov byte [ISAPNP_TEMP_CHECKSUM], 0x6A

next_encoded_bit:	
ISAPNP_read_data 	byte [ISAPNP_TEMP_DATA]	
mov al, [ISAPNP_TEMP_DATA]
shl ax, 8
mov [ISAPNP_TEMP_SIGNAL], ax
jmp short $+2		; Delay at least 250useconds 
ISAPNP_read_data 	byte [ISAPNP_TEMP_DATA]
xor ax, ax
mov al, [ISAPNP_TEMP_DATA]
add [ISAPNP_TEMP_SIGNAL], ax
jmp short $+2		; Delay at least 250useconds 
mov byte [ISAPNP_TEMP_DATA], 0
cmp word [ISAPNP_TEMP_SIGNAL], 0x55AA
jne skip_bit
mov byte [ISAPNP_TEMP_DATA], 1
mov eax, dword [ISAPNP_TEMP_VALUE]
mov ebx, dword [ISAPNP_TEMP_BITMASK]
or eax, ebx
mov dword [ISAPNP_TEMP_VALUE], eax
skip_bit:

;;; checksum = ((((checksum ^ (checksum >> 1)) & 0x01) ^ bit) << 7) | (checksum >> 1);
mov al, [ISAPNP_TEMP_CHECKSUM]
mov bl, al
shr bl, 1
xor al,bl 
and al, 1
xor al, byte [ISAPNP_TEMP_DATA] 
shl al, 7
or al, bl
mov [ISAPNP_TEMP_CHECKSUM], al

; Delay at least 1msecond
;TIMER_delay	MSEC_PIT_FREQUENCY, 1
TIMER_rtc_msdelay	1
cmp dword [ISAPNP_TEMP_BITMASK], 10000000000000000000000000000000b 
je next_32bit
shl dword [ISAPNP_TEMP_BITMASK], 1
jmp dword 0x0008:next_encoded_bit
next_32bit:
mov eax, dword [ISAPNP_TEMP_VALUE]
mov edi, dword [ISAPNP_TEMP_ADDRESS]
mov dword [ds:edi], eax
mov dword [ISAPNP_TEMP_VALUE], 0
add dword [ISAPNP_TEMP_ADDRESS], 4
mov dword [ISAPNP_TEMP_BITMASK], 1
cmp dword [ISAPNP_TEMP_ADDRESS], ISAPNP_SERIAL_NUMBER+10
je checksum_bit
jmp dword 0x0008:next_encoded_bit

checksum_bit:
ISAPNP_read_data 	byte [ISAPNP_TEMP_DATA]	
mov al, [ISAPNP_TEMP_DATA]
shl ax, 8
mov [ISAPNP_TEMP_SIGNAL], ax
jmp short $+2		; Delay at least 250useconds 
ISAPNP_read_data 	byte [ISAPNP_TEMP_DATA]
xor ax, ax
mov al, [ISAPNP_TEMP_DATA]
add [ISAPNP_TEMP_SIGNAL], ax
jmp short $+2		; Delay at least 250useconds 
cmp word [ISAPNP_TEMP_SIGNAL], 0x55AA
jne skip_bit1
mov eax, dword [ISAPNP_TEMP_VALUE]
mov ebx, dword [ISAPNP_TEMP_BITMASK]
or eax, ebx
mov dword [ISAPNP_TEMP_VALUE], eax
skip_bit1:
; Delay at least 1msecond
;TIMER_delay	MSEC_PIT_FREQUENCY, 1
TIMER_rtc_msdelay	1
cmp dword [ISAPNP_TEMP_BITMASK], 00000000000000000000000010000000b 
je check_checksum
shl dword [ISAPNP_TEMP_BITMASK], 1
jmp dword 0x0008:checksum_bit

check_checksum:
mov ebx, dword [ISAPNP_TEMP_VALUE]
mov edi, dword [ISAPNP_TEMP_ADDRESS]
mov byte [ds:edi], bl
mov al, [ISAPNP_TEMP_CHECKSUM]
cmp al, bl
jne end_isolation
;jmp dword 0x0008:end_isolation
;isolate:
inc byte [ISAPNP_CSN]
ISAPNP_set_card	 [ISAPNP_CSN]
jmp short $+2		; Delay at least 250useconds 
;TIMER_delay	MSEC_PIT_FREQUENCY, 1
ISAPNP_wakeup_csn	0x00
ISAPNP_serial_isolation
mov edi, ISAPNP_ENCODED_SIGNATURE
mov dword eax, '0000'
mov dword ebx, '0000'
mov dword [ds:edi], eax
mov dword [ds:edi+4], ebx
end_isolation:

retf

;
;   Bit value   |         Results of
;   E1  |   E2  | E1 & E2 | E1 ^ E2 | E1 | E2
;----------------------------------------------
;   0   |   0   |    0    |    0    |    0
;   1   |   0   |    0    |    1    |    1
;   0   |   1   |    0    |    1    |    1
;   1   |   1   |    1    |    0    |    1

%macro ISAPNP_get_encoded_char            0
; al = encoded number           
mov esi, ISAPNP_ENCODED_CHAR
mov bl, al
mov eax, 0
mov al, bl
add esi, eax
dec esi
mov al, byte [ds:esi]
; al = ASCII 
%endmacro

ISAPNP_detect:
; BIT 7 = 0
mov edi, ISAPNP_ENCODED_SIGNATURE
mov esi, ISAPNP_SERIAL_NUMBER+2
mov ax, word [ds:esi]
mov dx, 0000000001111100b
push ax
and ax, dx
shr ax, 2	; Shift to RIGHT
ISAPNP_get_encoded_char
mov byte [ds:edi], al
inc edi
pop ax
push ax
mov dx, 1110000000000011b
and ax, dx
rol ax, 3	; Rotate RIGHT-to-LEFT
ISAPNP_get_encoded_char
mov byte [ds:edi], al
inc edi
pop ax
mov dx, 0001111100000000b
and ax, dx
shr ax, 8 	; Shift to RIGHT
ISAPNP_get_encoded_char
mov byte [ds:edi], al
mov esi, ISAPNP_SERIAL_NUMBER+4
xor eax, eax
mov ax, word [ds:esi]
xchg al, ah
mov [ISAPNP_TEMP_VALUE], eax
number_to_text  dword [ISAPNP_TEMP_VALUE], 16, ISAPNP_ENCODED_SIGNATURE+6
mov edi, ISAPNP_ENCODED_SIGNATURE+7
mov byte [ds:edi], 0 

mov edi, ISAPNP_DEVICES
mov esi, ISAPNP_ENCODED_SIGNATURE
mov edx, 0
next_device:
mov al, byte [ds:esi+edx]
mov bl, byte [ds:edi+edx]
cmp al, bl
je next_char2
mov edx, 0
add edi, 38
cmp edi, END_ISAPNP_DEVICES
jne next_device
sub edi, 38
jmp short end_detect
next_char2:
inc edx
cmp edx, 7
jne next_device
end_detect:
add edi, 7 
mov dword [ISAPNP_CARD_STRING], edi 

retf


;============================================================================;
ISAPNP_CHECK_PORTS		DB      0x203, 0x20B, 0x213, 0x21B, 0x223, 0x22B 
				DB	0x233, 0x23B, 0x243, 0x24B, 0x253, 0x25B
				DB	0x263, 0x26B, 0x273, 0x27B, 0x283, 0x28B
				DB	0x293, 0x29B, 0x2A3, 0x2AB, 0x2B3, 0x2BB
				DB	0x2C3, 0x2CB, 0x2D3, 0x2DB, 0x2E3, 0x2EB
				DB	0x2F3, 0x2FB, 0x303, 0x30B, 0x313, 0x31B 
				DB	0x323, 0x32B, 0x333, 0x33B, 0x343, 0x34B
				DB	0x353, 0x35B, 0x363, 0x36B, 0x373, 0x37B
				DB	0x383, 0x38B, 0x393, 0x39B, 0x3A3, 0x3AB
				DB	0x3B3, 0x3BB, 0x3C3, 0x3CB, 0x3D3, 0x3DB
				DB	0x3E3, 0x3EB, 0x3F3, 0x3FB

ISAPNP_TEMP_DATA		DB	0
ISAPNP_TEMP_CHECKSUM		DB	0
ISAPNP_TEMP_SIGNAL		DW	0
ISAPNP_TEMP_VALUE		DD	0
ISAPNP_TEMP_ADDRESS		DD	0
ISAPNP_TEMP_BITMASK		DD	0
ISAPNP_WRITE_ADDRESS_PORT	equ 	0x279
ISAPNP_WRITE_DATA_PORT		equ	0xA79
ISAPNP_DEVICE_PORT		DW	0
ISAPNP_DEVICE_PORT_INC		equ	32
ISAPNP_DEVICE_PORT_START	equ	0x200
ISAPNP_DEVICE_PORT_END		equ	0x3FF
ISAPNP_INITKEY	 		DB	0x6A, 0xB5, 0xDA, 0xED, 0xF6, 0xFB
				DB	0x7D, 0xBE, 0xDF, 0x6F, 0x37, 0x1B
				DB	0x0D, 0x86, 0xC3, 0x61, 0xB0, 0x58 
				DB	0x2C, 0x16, 0x8B, 0x45, 0xA2, 0xD1
				DB	0xE8, 0x74, 0x3A, 0x9D, 0xCE, 0xE7
				DB	0x73, 0x39
ISAPNP_ENCODED_CHAR             DB 	'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
ISAPNP_CSN			DB	0
ISAPNP_SERIAL_NUMBER		DW	0		; port address
				DW	0010000100000100b	; encoded model name
				; initialize with AAA for debugging
				DW	0		; product number
				DD	0		; serial number
				DB	0		; checksum

ISAPNP_CARD_STRING		DD	0x00000000	; COOL Pointer !!! :)
ISAPNP_DEVICE_STRING		DD	0x00000000	; COOL Pointer !!! :)
ISAPNP_ENCODED_SIGNATURE	DB 	'        ',0
ISAPNP_DEVICES:			
;-----------------------------------------------------------------------;
;				PNP CARDS				;
;-----------------------------------------------------------------------;	
				DB	'CTL0001'	; Encoded Signature
				DB	'Creative Labs SB16            ',0
				DB	'CTL0021'
				DB	'Creative Labs AWE/AWE32       ',0
				DB	'CTL0022'
				DB	'Creative Labs AWE64 16-Bit    ',0
				DB	'CTL0023'
				DB	'Creative Labs AWE64 Gold      ',0
				DB	'CTL0024'
				DB	'Creative Labs AWE64           ',0
				DB	'CTL002B'
				DB	'Creative Labs SB16 PnP        ',0
				DB	'CTL0031'
				DB	'Creative Labs SB16            ',0
				DB	'CTL0041'
				DB	'Creative Labs V16CL/V32D/V32G ',0
				DB	'CTL0042'
				DB	'Creative Labs AWE64 WaveTable ',0
				DB	'CTL0043'
				DB	'Creative Labs SB16            ',0
				DB	'CTL0044'
				DB	'Creative Labs AWE64 Gold      ',0
				DB	'CTL0045'
				DB	'Creative Labs AWE64 16-Bit    ',0
				DB	'CTL0046'
				DB	'Creative Labs AWE64 16-Bit    ',0
				DB	'CTL0047'
				DB	'Creative Labs SB16            ',0
				DB	'CTL0051'
				DB	'Creative Labs 3D Stereo       ',0
				DB	'CTL0070'
				DB	'Creative Labs ViBRA16C        ',0
				DB	'CTL009D'
				DB	'Creative Labs SB AWE64        ',0
				DB	'CTL009E'
				DB	'Creative Labs SB AWE64 Gold   ',0
				DB	'CTL00C3'
				DB	'Creative Labs SB AWE64        ',0
				DB	'CTL00E4'
				DB	'Creative Labs SB AWE64        ',0
				DB	'CTL00F0'
				DB	'Creative Labs Vibra16X        ',0
				DB	'CTL1000'
				DB	'Creative Labs 3D Blaster      ',0
				DB	'CTL1003'
				DB	'Creative Labs 3D Blaster      ',0
				DB	'CTL1004'
				DB	'Creative Labs 3D Blaster      ',0
				DB	'CTL1005'
				DB	'Creative Labs 3D Blaster      ',0
				DB	'CTL1006'
				DB	'Creative Labs 3D Blaster      ',0
				DB	'CTL2001'
				DB	'Creative Labs IDE Interface   ',0
				DB	'CTL2002'
				DB	'Creative Labs IDE CD-ROM Drive',0
				DB	'CTL2011'
				DB	'Creative Labs IDE CD-ROM Drive',0
				DB	'CTL7001'
				DB	'Creative Labs Joystick Control',0
				DB	'CTL7001'
				DB	'Creative Labs Game Port       ',0
				DB	'CTL7002'
				DB	'Creative Labs Prog. Game Port ',0
				DB	'CTL7005'
				DB	'Creative Labs Prog. Game Port ',0
				DB	'CTL8001'
				DB	'Creative Labs Legacy SB       ',0
				DB	'CTL9011'
				DB	'Creative Labs RT300 VB        ',0

				DB	'PNPB000'
				DB	'Sound Blaster 1.5             ',0
				DB	'PNPB001'
				DB	'Sound Blaster 2.0             ',0
				DB	'PNPB002'
				DB	'Sound Blaster Pro             ',0
				DB	'PNPB003'
				DB	'Sound Blaster 16              ',0

				DB	'ICU0180'
				DB	'Sound Blaster                 ',0
				DB	'ICU0190'
				DB	'Sound Blaster Pro             ',0
				DB	'ICU01A0'
				DB	'Sound Blaster 16ASP           ',0
				DB	'ICU01A1'
				DB	'Sound Blaster 16              ',0
				DB	'ICU01A2'
				DB	'Sound Blaster 16 SCSI-2       ',0
				DB	'ICU01A3'
				DB	'Sound Blaster 16ASP/CD-ROM    ',0

				DB	'OPT0000'
				DB	'OPTi SoundExpression System   ',0
				DB	'OPT0001'
				DB	'OPTi SoundExpression Game Port',0
				DB	'OPT0002'
				DB	'OPTi SoundExpression MPU-401  ',0
				DB	'OPT0005'
				DB	'OPTi SoundExpression Control  ',0
				DB	'OPT0006'
				DB	'OPTi PNP COM Port             ',0
				DB	'OPT0007'
				DB	'OPTi PNP IDE Controller       ',0
				DB	'OPT0931'
				DB	'OPTi 86C931 Audio Card        ',0
				DB	'OPT9300'
				DB	'OPTi 930 Sound/OPL3 Device    ',0
				DB	'OPT9301'
				DB	'OPTi 930 MPU-401 Device       ',0
				DB	'OPT9310'
				DB	'OPTi PNP Sound System         ',0
				DB	'XXXXXXX'
				DB	'Unknown Device                ',0

END_ISAPNP_DEVICES:

;
; in the future this device ID information will be in a file
;
; TODO: Standard ISA/EISA Specification (Non-PNP Mode)
;
