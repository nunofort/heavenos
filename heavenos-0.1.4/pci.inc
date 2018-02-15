

%macro PCI_get_type	1
mov ebx, 2
mov ax, 0x0000
mov dx, PCI_ADDRESS_PORT
out dx, ax
in ax, dx 
cmp ax, 0x0000
jne %%type_1 
add dx, 2
out dx, ax
in ax, dx
sub dx, 2
cmp ax, 0x0000
je %%exit_type
%%type_1:
mov ebx, 1
mov eax, 0x80000000
out dx, eax
in eax, dx
cmp eax, 0x80000000
je %%exit_type
mov ebx, 0
%%exit_type:
mov %1, ebx
%endmacro

%macro PCI_get_device_info 	3
mov ebx, %1
cmp ebx, 1
jne %%type_2
xor ebx,ebx
mov bx, %2
shl ebx, 11
or ebx, 0x80000000
mov cx, 64
mov edi, %3
%%next_info1:
mov eax, ebx
mov dx, PCI_ADDRESS_PORT
out dx, eax
mov dx, PCI_DATA_PORT
in eax, dx
mov dword [es:edi], eax
add edi, 4
add ebx, 4
dec cx
jnz %%next_info1
jmp short %%exit_info
%%type_2: 
mov ebx, %1
cmp ebx, 2
jne %%exit_info
mov dx, PCI_ADDRESS_PORT
mov al, 0x80
out dx, al
xor al, al
add dx, 2
out dx, al
sub dx, 2
mov dx, %2
shl dx, 8
or dx, 0xC000
mov cx, 64
mov edi, %3
%%next_info2:
in eax, dx
mov dword [es:edi], eax
add edi, 4
add dx, 4
dec cx
jnz %%next_info2
mov dx, PCI_ADDRESS_PORT
xor al, al
out dx, al
%%exit_info:
%endmacro

%macro PCI_detect_device	4	
mov ebx, %1
mov esi, PCI_DEVICES
%%next_device:
mov eax, dword [es:esi]
add esi, 35 
cmp eax, ebx
je %%exit_devices
cmp esi, END_PCI_DEVICES
jne %%next_device
%%exit_devices:
sub esi, 31
mov %3, esi
mov bl, %2 
mov esi, PCI_CLASSES
%%next_class:
mov al, byte [es:esi]
add esi, 22
cmp al, bl
je %%exit_classes
cmp esi, END_PCI_CLASSES
jne %%next_class
%%exit_classes:
sub esi, 21
mov %4, esi
%endmacro

;=============================================================================;
PCI_ADDRESS_PORT	equ     0xCF8
PCI_DATA_PORT		equ     0xCFC
pci_type		DD	0
pci_device_string	DD	0x00000000 ; COOL Pointer!!! :)
pci_class_string	DD	0x00000000 ; COOL Pointer!!! :)
;------------------------------------------------------------------------------
;				PCI DEVICE STRUCTURE 
;------------------------------------------------------------------------------
pci_device		DW	0x0000		; 00h Vendor ID
			DW	0x0000		; 02h Device ID
			DW	0x0000		; 04h Bus Command
			DW	0x0000		; 06h Bus Status
			DB	0x00		; 08h Revision Code
			DB	0x00		; 09h Prog Interface
			DW	0x0000		; 0Ah Class Code
			DB	0x00		; 0Ch Cache Line Size
			DB	0x00		; 0Dh Latency Timer
			DB	0x00		; 0Eh Header Type
			DB	0x00		; 0Fh ist
			DD	0x00000000	; 10h Base Address 0
			DD	0x00000000	; 14h Base Address 1
			DD	0x00000000	; 18h Base Address 2
			DD	0x00000000	; 1Ch Base Address 3
			DD	0x00000000	; 20h Base Address 4
			DD	0x00000000	; 24h Base Address 5
			DD	0x00000000	; 28h Reserved1
			DD	0x00000000	; 2Ch Reserved2
			DD	0x00000000	; 30h ROM Base Address
			DD	0x00000000	; 34h Reserved3
			DD	0x00000000	; 38h Reserved4
			DB	0x00		; 3Ch Interrupt Line 
			DB	0x00		; 3Dh Interrupt Pin
			DB 	0x00		; 3Eh Min Grant
			DB	0x00		; 3Fh Max Latency
times 	192		DB 	0x00		; 40h Reserved5 (for PCI Unit)
PCI_CLASSES		
;------------------------------------------------------------------------------
;				PCI CLASS 
;------------------------------------------------------------------------------
			DB	0x00,'Unknown Class       ',0
			DB	0x01,'Mass Storage Device ',0
			DB	0x02,'Network Device      ',0
			DB	0x03,'Video Device        ',0
			DB	0x04,'Multimedia Device   ',0
			DB	0x05,'Memory Device       ',0
			DB	0x06,'Bridge Device       ',0
			DB	0xFF,'No Class for Device ',0
END_PCI_CLASSES:
PCI_DEVICES            
;------------------------------------------------------------------------------
;				VIDEO CARDS
;------------------------------------------------------------------------------
    			DW 0x5333,0x0081
			DB 'S3 Inc, 911                   ',0
			DW 0x5333,0x0082
			DB 'S3 Inc, 911A/924              ',0
			DW 0x5333,0x0090
			DB 'S3 Inc, 928                   ',0
			DW 0x5333,0x0091
			DB 'S3 Inc, 928                   ',0
			DW 0x5333,0x0092
			DB 'S3 Inc, 928                   ',0
			DW 0x5333,0x0093
			DB 'S3 Inc, 928                   ',0
			DW 0x5333,0x0094
			DB 'S3 Inc, 928                   ',0
			DW 0x5333,0x0095
			DB 'S3 Inc, 928                   ',0
			DW 0x5333,0x00A0
			DB 'S3 Inc, 801/805               ',0
			DW 0x5333,0x00A1
			DB 'S3 Inc, 801/805               ',0
			DW 0x5333,0x00A2
			DB 'S3 Inc, 801/805               ',0
			DW 0x5333,0x00A3
			DB 'S3 Inc, 801/805               ',0
			DW 0x5333,0x00A4
			DB 'S3 Inc, 801/805               ',0
			DW 0x5333,0x00A5
			DB 'S3 Inc, 801/805               ',0
			DW 0x5333,0x00A6
			DB 'S3 Inc, 801/805               ',0
			DW 0x5333,0x00A7
			DB 'S3 Inc, 801/805               ',0
			DW 0x5333,0x00A8
			DB 'S3 Inc, 801/805               ',0
			DW 0x5333,0x00B0
			DB 'S3 Inc, 928PCI                ',0
			DW 0x5333,0x00C0
			DB 'S3 Inc, Vision 864            ',0
			DW 0x5333,0x00C1
			DB 'S3 Inc, Vision 864            ',0
			DW 0x5333,0x00D0
			DB 'S3 Inc, Vision 964            ',0
			DW 0x5333,0x00D1
			DB 'S3 Inc Vision 964             ',0
			DW 0x5333,0x0551
			DB 'S3 Inc, PLATO/PX              ',0
			DW 0x5333,0x5631
			DB 'S3 Inc, 86c325 (ViRGE)        ',0
			DW 0x5333,0x8800
			DB 'S3 Inc, Vision 866            ',0
			DW 0x5333,0x8802
			DB 'S3 Inc, Vision 968            ',0
			DW 0x5333,0x8810
			DB 'S3 Inc, 86C732 (Trio32) Rev 00',0
			DW 0x5333,0x8811
			DB 'S3 Inc, 86c764 (Trio64)       ',0
			DW 0x5333,0x8812
			DB 'S3 Inc, 86cm65 (Aurora64V+)   ',0
			DW 0x5333,0x8814
			DB 'S3 Inc, 86c767 (Trio64UV+)    ',0
			DW 0x5333,0x8815
			DB 'S3 Inc, 86cm65 (Aurora128)    ',0
			DW 0x5333,0x883D
			DB 'S3 Inc, 86c988 (Virge/VX)     ',0
			DW 0x5333,0x8880
			DB 'S3 Inc, 86c866 (Vision)       ',0
			DW 0x5333,0x8890
			DB 'S3 Inc, 86c868 (Vision)       ',0
			DW 0x5333,0x88B0
			DB 'S3 Inc, 86c928P (Vision)      ',0
			DW 0x5333,0x88C0
			DB 'S3 Inc, 86c864 (Vision)       ',0
			DW 0x5333,0x88C1
			DB 'S3 Inc, 86c864P (Vision)      ',0
			DW 0x5333,0x88D0
			DB 'S3 Inc, 86c964 (Vision)       ',0
			DW 0x5333,0x88D1
			DB 'S3 Inc, 86c964P (Vision)      ',0
			DW 0x5333,0x88F0
			DB 'S3 Inc, 86c968 (Vision)       ',0
			DW 0x5333,0x8900
			DB 'S3 Inc, 86C775 (Trio64V2/DX)  ',0
			DW 0x5333,0x8901
			DB 'S3 Inc, Trio64V2 DX/GX        ',0
			DW 0x5333,0x8902
			DB 'S3 Inc, Plato PX              ',0
			DW 0x5333,0x8904
			DB 'S3 Inc, Trio 3D               ',0
			DW 0x5333,0x8A01
			DB 'S3 Inc, Virge DX/GX           ',0
			DW 0x5333,0x8A10
			DB 'S3 Inc, ViRGE/GX2             ',0
			DW 0x5333,0x8A13
			DB 'S3 Inc, Trio3D/2x/2x+         ',0
			DW 0x5333,0x8A20
			DB 'S3 Inc, 86c391 (Savage3D)     ',0
			DW 0x5333,0x8A21
			DB 'S3 Inc, 86c390 (Savage3D/MV)  ',0
			DW 0x5333,0x8A22
			DB 'S3 Inc, 86C394-397 (Savage4)  ',0
			DW 0x5333,0x8C00
			DB 'S3 Inc, 86C260 M3 (ViRGE/MX)  ',0
			DW 0x5333,0x8C01
			DB 'S3 Inc, ViRGE/MX              ',0
			DW 0x5333,0x8C02
			DB 'S3 Inc, 86C240 (ViRGE/MXC)    ',0
			DW 0x5333,0x8C03
			DB 'S3 Inc, ViRGE/MX+             ',0
			DW 0x5333,0x8C10
			DB 'S3 Inc,SavageMX/IX/MX+MV/IX+MV',0
			DW 0x5333,0x8C12
			DB 'S3 Inc,SavageMX/IX/MX+MV/IX+MV',0
			DW 0x5333,0x9102
			DB 'S3 Inc, 86c410 (Savage2000)   ',0
			DW 0x5333,0xCA00
			DB 'S3 Inc, 86c617 (SonicVibes)   ',0
			DW 0x10DE,0x002D
			DB 'NVidia RIVA TNT2 Model 64     ',0
			DW 0x121A,0x0001
			DB '3Dfx Inc, Voodoo Chipset      ',0
			DW 0x121A,0x0002
			DB '3Dfx Inc, Voodoo2 Chipset     ',0
			
; 10DEh - - NVidia Corporation
; 002Dh RIVA TNT2 Model 64
; 121Ah - - 3Dfx Interactive, Inc
; 0001h 3Dfx Voodoo Chipset
; 0002h 3Dfx Voodoo2 Chipset
;------------------------------------------------------------------------------
;				BRIDGE
;------------------------------------------------------------------------------
			DW 0xFFFF,0xFFFF
			DB 'Unknown Device                ',0			
END_PCI_DEVICES:

;
; in the future this device ID information will be in a file
;

