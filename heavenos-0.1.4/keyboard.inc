;
;^8042 - Keyboard Controller  (AT,PS/2)
;                    
;%       8042 Status Register (port 64h read)
;                    
;        |7|6|5|4|3|2|1|0|  8042 Status Register
;         | | | | | | | ----- output register (60h) has data for system
;         | | | | | | ------ input register (60h/64h) has data for 8042
;         | | | | | ------- system flag (set to 0 after power on reset)
;         | | | | -------- data in input register is command (1) or data (0)
;         | | | --------- 1=keyboard enabled, 0=keyboard disabled (via switch)
;         | | ---------- 1=transmit timeout (data transmit not complete)
;         | ----------- 1=receive timeout (data transmit not complete)
;         ------------ 1=even parity rec'd, 0=odd parity rec'd (should be odd)
;        
;%       Port Mode                 Description
;              
;        64h  read   8042 status register. Can be read at any time.  See
;                    table above for more information.
;        64h  write  8042 command register.  Writing this port sets Bit 3
;                    of the status register to 1 and the byte is treated
;                    as a controller command.  Devices attached to the
;                    8042 should be disabled before issuing commands that
;                    return data since data in the output register will
;                    be overwritten.
;        60h  read   8042 output register (should only be read if Bit 0 of
;                    status port is set to 1)
;        60h  write  8042 data register.  Data should only be written if
;                    Bit 1 of the status register is zero (register is empty).
;                    When this port is written Bit 3 of the status register
;                    is set to zero and the byte is treated as a data.  The
;                    8042 uses this byte if it's expecting data for a previous
;                    command, otherwise the data is written directly to the
;                    keyboard.   See ~KEYBOARD COMMANDS~ for information on
;                    programming the actual keyboard hardware.
;         
;         
;^8042 Commands Related to PC Systems  (Port 64h)
;         
;%       Command                    Description
;        
;         20   Read 8042 Command Byte: current 8042 command byte is placed
;              in port 60h.
;         60   Write 8042 Command Byte: next data byte written to port 60h is
;              placed in 8042 command register.  Format:
;        
;             |7|6|5|4|3|2|1|0|  8042 Command Byte
;              | | | | | | | ----- 1=enable output register full interrupt
;              | | | | | | ------ should be 0 
;              | | | | | ------- 1=set status register system, 0=clear
;              | | | | -------- 1=override keyboard inhibit, 0=allow inhibit
;              | | | --------- disable keyboard I/O by driving clock line low
;              | | ---------- disable auxiliary device, drives clock line low
;              | ----------- IBM scancode translation 0=AT, 1=PC/XT
;              ------------ reserved, should be 0
;                    
;         A4   Password Installed Test: returned data can be read
;              from port 60h;  FA=password installed, F1=no password
;         A5   Load Security: bytes written to port 60h will be read
;              until a null (0) is found.
;         A6   Enable Security: works only if a password is already loaded
;         A7   Disable Auxiliary Interface: sets Bit 5 of command register
;              stopping auxiliary I/O by driving the clock line low
;         A8   Enable Auxiliary Interface: clears Bit 5 of command register
;         A9   Auxiliary Interface Test: clock and data lines are tested;
;              results placed at port 60h are listed below:
;        
;                00  no error
;                01  keyboard clock line is stuck low
;                02  keyboard clock line is stuck high
;                03  keyboard data line is stuck low
;                04  keyboard data line is stuck high
;             
;         AA   Self Test: diagnostic result placed at port 60h, 55h=OK
;         AB   Keyboard Interface Test:  clock and data lines are tested;
;              results placed at port 60h are listed above with command A9
;         AC   Diagnostic Dump: sends 16 bytes of 8042's RAM, current input
;              port state, current output port state and 8042 program status
;              word to port 60h in scan-code format.
;         AD   Disable Keyboard Interface: sets Bit 4 of command register
;              stopping keyboard I/O by driving the clock line low
;         AE   Enable Keyboard Interface: clears Bit 4 of command register
;              enabling keyboard interface.
;         C0   Read Input Port: data is read from its input port (which is
;              inaccessible to the data bus) and written to output register
;              at port 60h;  output register should be empty before call.
;         
;               |7|6|5|4|3-0|  8042 Input Port
;                | | | | ----- undefined
;                | | | ------ 1=enable 2nd 256K of motherboard RAM, 0=disable
;                | | ------- 1=manufacturing jumper not installed, 0=installed
;                | -------- 1=primary display is MDA, 0=primary display is CGA
;                --------- 1=keyboard not inhibited, 0=keyboard inhibited
;                
;         C1   Poll Input Port Low Bits: Bits 0-3 of port 1 placed in
;              status Bits 4-7
;         C2   Poll Input Port High Bits: Bits 4-7 of port 1 placed in
;              status Bits 4-7
;         D0   Read Output Port: data is read from 8042 output port (which is
;              inaccessible to the data bus) and placed in output register;
;              the output register should be empty.  (see command D1 below)
;         D1   Write Output Port: next byte written to port 60h is placed in
;              the 8042 output port (which is inaccessible to the data bus)
;              
;                |7|6|5|4|3|2|1|0|  8042 Output Port
;                 | | | | | | | ----- system reset line
;                 | | | | | | ------ gate A20
;                 | | | | --------- undefined
;                 | | | ---------- output buffer full
;                 | | ----------- input buffer empty
;                 | ------------ keyboard clock (output)
;                 ------------- keyboard data (output)
;         
;         D2   Write Keyboard Output Register: on PS/2 systems the next data
;              byte written to port 60h input register is written to port 60h
;              output register as if initiated by a device; invokes interrupt
;              if enabled
;         D3   Write Auxiliary Output Register: on PS/2 systems the next data
;              byte written to port 60h input register is written to port 60h
;              output register as if initiated by a device; invokes interrupt
;              if enabled
;         D4   Write Auxiliary Device: on PS/2 systems the next data byte
;              written to input register a port at 60h is sent to the
;              auxiliary device
;         E0   Read Test Inputs: 8042 reads its T0 and T1 inputs; data is
;              placed in output register;  Bit 0 is T0, Bit 1 is T1:
;                |1|0|  Test Input Port Bits
;                 | ----- keyboard clock
;                 ------ keyboard data
;                
;         Fx   Pulse Output Port: Bits 0-3 of the 8042 output port can be
;              pulsed low for 6 ms;  Bits 0-3 of command indicate which
;              Bits should be pulsed; 0=pulse, 1=don't pulse; pulsing
;              Bit 0 results in CPU reset since it is connected to system
;              reset line.
;                 
;        - PC systems previous to the AT use the 8255 PPI as a keyboard
;          controller and use the keyboard's internal 8048.
;        - the keyboard's internal controller buffers up to 16 bytes of
;          make/break code information.  This is common among all PC systems
;          and shouldn't be confused with the (32 byte) keyboard buffer
;          maintained by the BIOS.
;        - see  ~KEYBOARD COMMANDS~ for information on programming the
;          keyboards internal microprocessor
;
; KEYBOARD COMMANDS
;
;EDh nn          write LEDs, as above
;EEh             echo, keyboard responds with EEh
;EFh             no-operation (reserved)
;F0h nn          selects scancode set nn=1-3 or 0 to return current set
;F2h             read ID. Keyboard responds with ACK (FAh) and two optional
;                ID bytes:
;                    (none)      AT keyboard
;                    83h ABh     (?)
;                    ABh 41h     MF2, translation mode
;                    ABh 83h     MF2, pass-through mode
;F3h nn          set typematic (auto-repeat) rate/delay. nn=
;                    b7      unused
;                    b6..5   Repeat delay (00=250 msec ... 11=1000msec)
;                    b4..0   Repeat rate (00000=30 Hz ... 11111=2 Hz).
;F4h             clears output buffer, enables keyboard
;F5h             disables keyboard, resets to defaults
;F6h             sets keyboard defaults
;
;F7h             make all keys typematic (auto-repeat)   [*]
;F8h             make all keys make-break                [*]
;F9h             make all keys make-only                 [*]
;FAh             make all keys typematic and make-break  [*]
;FBh nn          make one key typematic                  [*]
;FCh nn          make one key make-break                 [*]
;FDh nn          make one key make-only                  [*]
;                        [*] these commands may work only for
;                            scancode set 3; I'm not sure.
;FEh             resend previous scan code
;FFh             reset keyboard CPU, do power-on self-test, return
;                self-test result byte
;


%macro KEYBOARD_write	2
%%get_status_write:
in al, KEYBOARD_COMMAND_PORT
mov ah, 0x02
and ah, al
cmp ah, 0x02
je %%get_status_write	; loop until 8042 input buffer empty
xor dx, dx
mov dl, %1
mov al, %2
out dx, al
%endmacro

%macro KEYBOARD_read	1
%%get_status_read:
in al, KEYBOARD_COMMAND_PORT
mov ah, 0x01
and ah, al
cmp ah, 0x01
jne %%get_status_read	; loop until 8042 output buffer full
in al, KEYBOARD_DATA_PORT
mov ah, 0xC0
and ah, al
cmp ah, 0
jne %%get_status_read	; loop if parity error
mov byte [%1], al
%endmacro

%macro KEYBOARD_init	0
KEYBOARD_write		0x60, 0xF5	; Disable Keyboard
; Expect 0xFA 
KEYBOARD_write		0x64, 0x60
KEYBOARD_write		0x60, 01100101b	; Enable Interrupt and Disable PS/2 Mouse
KEYBOARD_write		0x60, 0xF6	; Keyboard Defaults
;;;KEYBOARD_write		0x60, 0xF0	; Setting Scancode
; Expect 0xFA
;;;KEYBOARD_write		0x60, 0x01	; Scancode Set
; Expect 0xFA
KEYBOARD_write		0x60, 0xF7	; Make all Keys Typematic (auto-repeat)
; Expect 0xFA
KEYBOARD_write		0x60, 0xF3	; Setting Typematic Delay
; Expect 0xFA
KEYBOARD_write		0x60, 0		; RATE=30 chars/s,DELAY=250 msec
; Expect 0xFA
KEYBOARD_write		0x60, 0xF4	; Enable Keyboard
; Expect 0xFA
%endmacro

%macro KEYBOARD_get_key		1	
%%get_again:
xor	ax, ax
mov	al, [KEYBOARD_KEY]
cmp	ax, KEYBOARD_RELEASE_KEY	; !!! This MUST be a 16-bit register !!!
jge %%get_again
mov	byte [%1], al
mov	al, KEYBOARD_RELEASE_KEY
mov 	[KEYBOARD_KEY], al
%endmacro

%macro  KEYBOARD_check_key  	1
mov	bl, %1
%%repeat_check:
;;;in      al, KEYBOARD_IO_PORT    ; 0x60 IO PORT
mov	al, [KEYBOARD_KEY]
cmp     al, bl                  ; Key to check if is pressed
jne     %%repeat_check          ; JUMP if not equal
xor	al, al
mov 	[KEYBOARD_KEY], al
%endmacro

%macro KEYBOARD_direct_check_key	1
mov     bl, %1
%%repeat_check:
in      al, 0x60                ; GET KEY FROM KEYBOARD
cmp     al, bl                  ; CHECK KEY
jne     %%repeat_check          ; JUMP if not equal
%endmacro

%macro KEYBOARD_reset	0
mov     al, 0xFF
out     KEYBOARD_DATA_PORT, al            ; 0x60 IO PORT
%endmacro

%macro  KEYBOARD_status 	1
in      al, KEYBOARD_COMMAND_PORT       ; 0x64 CONTROL PORT
mov 	byte [%1], al
%endmacro

%macro KEYBOARD_clear_buffer 0
mov esi, 16 * 0x0040 + 0x001C
mov edi, 16 * 0x0040 + 0x001A
mov al, byte [es:esi]
mov byte [es:edi], al
%endmacro

%macro KEYBOARD_selftest	0
mov     al, 0xAA
out     KEYBOARD_COMMAND_PORT, al       ; 0x64 CONTROL PORT
%endmacro

%macro KEYBOARD_interface_self_test	0
mov     al, 0xAB
out     KEYBOARD_COMMAND_PORT, al       ; 0x64 CONTROL PORT
%endmacro

%macro KEYBOARD_enable	0
mov     al, 0xAE
out     KEYBOARD_COMMAND_PORT, al       ; 0x64 CONTROL PORT
%endmacro

%macro KEYBOARD_disable	0
mov     al, 0xAD
out     KEYBOARD_COMMAND_PORT, al       ; 0x64 CONTROL PORT
%endmacro

KEYBOARD_system_reset:
mov al, 0xFE
out KEYBOARD_COMMAND_PORT, al

%macro KEYBOARD_map	1
mov esi, pt_keys
mov bl, [%1]
%%try_key:
inc esi
mov al, byte [ds:esi]
add esi, 2
cmp al, bl
je %%skip_keys
cmp esi, end_pt_keys 
jne %%try_key
%%skip_keys:
sub esi, 1
mov al, byte [ds:esi]
mov byte [%1], al
%endmacro

;==============================================================================
KEYBOARD_DATA_PORT        	equ     0x60
KEYBOARD_COMMAND_PORT    	equ     0x64
KEYBOARD_ESC_KEY		equ	0x01
KEYBOARD_ENTER_KEY		equ	0x1C
KEYBOARD_BACKSPACE_KEY		equ	14
KEYBOARD_RELEASE_KEY		equ	128	
KEYBOARD_KEY			DB	0x00	
;------------------------------------------------------------------------------
;                        KEYBOARD LAYOUT SCANCODES
;------------------------------------------------------------------------------
;
; Portuguese
; 		     press,   key,   ascii
pt_keys:	DB	0, 	1, 	0		; ESC
		DB	0, 	2, 	49		; 1
		DB	42, 	2, 	33		; Shift-1 = !	
		DB	0, 	3, 	50		; 2 
		DB	42, 	3, 	34		; Shift-2 = "
		DB	0, 	4, 	51		; 3
		DB	42, 	4, 	35		; Shift-3 = #
		DB	0, 	5, 	52		; 4
		DB	42, 	5, 	36		; Shift-4 = $
		DB	0, 	6, 	53		; 5
		DB	42, 	6, 	37		; Shift-5 = %
		DB	0, 	7, 	54		; 6
		DB	42, 	7, 	38		; Shift-6 = &
		DB	0, 	8, 	55		; 7
		DB	42, 	8, 	47		; Shift-7 = /
		DB	0, 	9, 	56		; 8
		DB	42, 	9, 	40		; Shift-8 = (
		DB 	0, 	10, 	57		; 9	
		DB 	42,	10, 	41		; Shift-9 = )
		DB 	0, 	11, 	48		; 0
		DB 	42, 	11, 	61		; Shift-0 = =
		DB 	0, 	14, 	0		; BACKSPACE
		DB 	0, 	16, 	113		; q
		DB 	42, 	16, 	81		; Q
		DB 	0, 	17, 	119		; w
		DB 	42, 	17, 	87		; W
		DB 	0, 	18, 	101		; e
		DB 	42, 	18, 	69		; E
		DB 	0, 	19, 	114		; r
		DB 	42, 	19, 	82		; R
		DB 	0, 	20, 	116		; t
		DB 	42, 	20, 	84		; T
		DB 	0, 	21, 	121		; y
		DB 	42, 	21, 	89		; Y
		DB 	0, 	22, 	117		; u
		DB 	42, 	22, 	85		; U
		DB 	0, 	23, 	105		; i
		DB 	42, 	23, 	73		; I
		DB 	0, 	24, 	111		; o
		DB 	42, 	24, 	79		; O
		DB 	0, 	25, 	112		; p
		DB 	42, 	25, 	80		; P 
		DB 	0, 	28, 	0		; ENTER
		DB 	0, 	29, 	0		; CTRL
		DB 	0, 	30, 	97		; a 
		DB 	42, 	30, 	65		; A
		DB 	0, 	31, 	115		; s
		DB 	42, 	31, 	83		; S
		DB 	0, 	32, 	100		; d
		DB 	42, 	32, 	68		; D
		DB 	0, 	33, 	102		; f
		DB 	42, 	33, 	70		; F
		DB 	0, 	34, 	103		; g
		DB 	42, 	34, 	71		; G
		DB 	0, 	35, 	104		; h
		DB 	42, 	35, 	72		; H
		DB 	0, 	36, 	106		; j
		DB 	42, 	36, 	74		; J
		DB 	0, 	37, 	107		; k
		DB 	42, 	37, 	75		; K
		DB 	0, 	38, 	108		; l
		DB 	42, 	38, 	76		; L
		DB 	0, 	42, 	0		; LEFT SHIFT
		DB 	0, 	43, 	0		; \
		DB 	42, 	43, 	0		; Shift-\ = |
		DB 	0, 	44, 	122		; z
		DB 	42, 	44, 	90		; Z
		DB 	0, 	45, 	120		; x
		DB 	42, 	45, 	88		; X
		DB 	0, 	46, 	99		; c
		DB 	42, 	46, 	67		; C
		DB 	0, 	47, 	118		; v
		DB 	42, 	47, 	86		; V
		DB 	0, 	48, 	98		; b
		DB 	42, 	48, 	66		; B
		DB 	0, 	49, 	110		; n 
		DB 	42, 	49, 	78		; N
		DB 	0, 	50, 	109		; m
		DB 	42, 	50, 	77		; M
		DB 	0, 	56, 	0		; ALT
		DB 	0, 	57, 	0		; SPACEBAR
		DB 	0, 	59, 	0		; F1
		DB 	0, 	60, 	0		; F2
		DB 	0, 	61, 	0		; F3
		DB 	0, 	62, 	0		; F4
		DB 	0, 	63, 	0		; F5
		DB 	0, 	64, 	0		; F6
		DB 	0, 	65, 	0		; F7
		DB 	0, 	66, 	0		; F8
		DB 	0, 	67, 	0		; F9
		DB 	0, 	68, 	0		; F10
		DB 	0, 	133, 	0		; F11
		DB 	0, 	134, 	0		; F12
		
end_pt_keys:


