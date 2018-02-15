
;                    DMA Channel   Page    Address  Count
;                    ------------------------------------
;                         0         87h       0h      1h
;                         1         83h       2h      3h
;                         2         81h       4h      5h
;                         3         82h       6h      7h
;                         4         8Fh      C0h     C2h
;                         5         8Bh      C4h     C6h
;                         6         89h      C8h     CAh
;                         7         8Ah      CCh     CEh
;
;      (Note: Channels 4-7 are 16-bit DMA channels.  See below for more info.)

;
; REGISTER (08h/D0h)
;
; Status Register
; 7-bit -> (1=DMA request/0=no DMA request) of channel (3/7)
; 6-bit -> (1=DMA request/0=no DMA request) of channel (2/6)
; 5-bit -> (1=DMA request/0=no DMA request) of channel (1/5)
; 4-bit -> (1=DMA request/0=no DMA request) of channel (0/4)
; 3-bit -> (1=transfer complete/0=transfer not complete) of channel (3/7)
; 2-bit -> (1=transfer complete/0=transfer not complete) of channel (2/6)
; 1-bit -> (1=transfer complete/0=transfer not complete) of channel (1/5)
; 0-bit -> (1=transfer complete/0=transfer not complete) of channel (0/4)
;
; Command Register ( byte )
; 7-bit -> 0 active low(standard)		1 active high
; 6-bit -> 0 active high(standard)		1 active low
; 5-bit -> 0 late write(standard)		1 extended write
; 4-bit -> 0 rotating priority			1 fixed priority(standard)
; 3-bit -> 0 normal mode(standard)		1 compressed mode
; 2-bit -> 0 DMA controller enabled		1 DMA controller disabled
; 1-bit -> 0 Address Hold disabled(standard) 	1 Address Hold enabled
; 0-bit -> 0 No (from/to)Memory transfer(standard)  1 (from/to)Memory transfer  
; For Memory-to-Memory transfers only channels 0(source) and 1(target) 
; are available 
;

%macro DMA_status	1
xor dx, dx
mov dl, [DMA_COMMAND_REGISTER]
in al, dx
mov %1, al
%endmacro

%macro DMA_command	1
xor dx, dx
mov dl, [DMA_COMMAND_REGISTER]
mov al, %1
out dx, al
%endmacro

%macro DMA_disable	0
DMA_command 	00010100b 
%endmacro

%macro DMA_enable	0
DMA_command	00010000b 
%endmacro

;DMA Write Request Register (09h/D2h):
;==================================
;
;    MSB                             LSB
;      x   x   x   x     x   x   x   x
;      -------------------   -   -----
;               |            |     |     00 - Select channel 0
;               |            |     \---- 01 - Select channel 1
;               |            |           10 - Select channel 2
;               |            |           11 - Select channel 3
;               |            |
;               |            \----------  0 - Device turn it on
;               |                         1 - Needs to be turned on
;               |                             (when memory-to-memory)
;               \----------------------- xx - Don't care
;

%macro DMA_request	1
xor dx, dx
mov dl, [DMA_REQUEST_REGISTER]
mov al, %1
out dx, al
%endmacro

;Mask Register (0Ah/D4h):
;=====================
;
;    MSB                             LSB
;      x   x   x   x     x   x   x   x
;      -------------------   -   -----
;               |            |     |     00 - Select channel 0/4 mask bit
;               |            |     \---- 01 - Select channel 1/5 mask bit
;               |            |           10 - Select channel 2/6 mask bit
;               |            |           11 - Select channel 3/7 mask bit
;               |            |
;               |            \----------  0 - Clear mask bit
;               |                         1 - Set mask bit
;               |
;               \----------------------- xx - Don't care
;

%macro DMA_mask		0
xor dx, dx
mov dl, [DMA_MASK_REGISTER]
out dx, al
%endmacro

%macro DMA_disable_channel	1
mov al, %1
or al, 0x04
DMA_mask	
%endmacro

%macro DMA_enable_channel	1
mov al, %1
DMA_mask
%endmacro

%macro DMA_set_mode	4
mov al, 0
or al, %1
or al, %2
or al, %3
or al, %4
or al, [DMA_CHANNEL]
mov [DMA_MODE], al
%endmacro

;Mode Register (0Bh/D6h):
;=====================
;
;    MSB                             LSB
;      x   x   x   x     x   x   x   x
;      \---/   -   -     -----   -----
;        |     |   |       |       |     00 - Channel 0/4 select
;        |     |   |       |       \---- 01 - Channel 1/5 select
;        |     |   |       |             10 - Channel 2/6 select
;        |     |   |       |             11 - Channel 3/7 select
;        |     |   |       |
;        |     |   |       |             00 - Verify transfer
;        |     |   |       \------------ 01 - Write transfer
;        |     |   |                     10 - Read transfer
;        |     |   |
;        |     |   \-------------------- 0 - Non-autoinitialized
;        |     |                         1 - Autoinitialized 
;        |     |                         
;        |     \------------------------  0 - Address increment select
;        |
;        |                               00 - Demand mode
;        \------------------------------ 01 - Single mode
;                                        10 - Block mode
;					 11 - Cascade mode
;

%macro DMA_mode		1
xor dx, dx
mov dl, [DMA_MODE_REGISTER]
mov al, %1
out dx, al
%endmacro

;DMA clear selected channel (0Ch/D8h):
;==================================
;
;    Outputting a zero to this port stops all DMA processes that are currently
;happening as selected by the mask register(0Ah/D4h).
;

%macro DMA_clear	1
xor dx, dx
mov dl, [DMA_CLEAR_REGISTER]
mov al, %1
mov ax, 0
out dx, ax 
%endmacro

%macro DMA_page		1
xor dx, dx
mov dl, [DMA_PAGE_REGISTER]
mov eax, %1
shr eax, 16
;;;and eax, 0xFF ; get 8-bit segment(page) value
out dx, al
%endmacro

%macro DMA_address	1
xor dx, dx
mov dl, [DMA_ADDRESS_REGISTER]
mov eax, %1
;;;and eax, 0xFFFF ; get 16-bit offset value
out dx, al
mov al, ah
out dx, al
%endmacro

%macro DMA_count	1
xor dx, dx
mov dl, [DMA_COUNT_REGISTER]
mov ax, %1
dec ax
out dx, al
mov al, ah
out dx, al
%endmacro

%macro DMA_set_registers        1
mov esi, DMA_REGISTERS
mov eax, 3
xor ebx, ebx
mov bl, %1
mul ebx
add esi, eax
mov al, byte [ds:esi]
mov [DMA_ADDRESS_REGISTER], al
mov al, byte [ds:esi+1]
mov [DMA_COUNT_REGISTER], al
mov al, byte [ds:esi+2]
mov [DMA_PAGE_REGISTER], al
;;; DMA 1 (8-bit)
mov byte [DMA_COMMAND_REGISTER], 0x08
mov byte [DMA_REQUEST_REGISTER], 0x09
mov byte [DMA_MASK_REGISTER], 0x0A
mov byte [DMA_MODE_REGISTER], 0x0B
mov byte [DMA_CLEAR_REGISTER], 0x0C
cmp bl, 4
jl %%skip_set
;;; DMA 2 (16-bit)
mov byte [DMA_COMMAND_REGISTER], 0xD0
mov byte [DMA_REQUEST_REGISTER], 0xD2
mov byte [DMA_MASK_REGISTER], 0xD4
mov byte [DMA_MODE_REGISTER], 0xD6
mov byte [DMA_CLEAR_REGISTER], 0xD8
sub byte [DMA_CHANNEL], 0x04
%%skip_set:
%endmacro

; CHARNEL
; BUFFER ADDRESS
; BUFFER LENGTH
%macro DMA_setup	3
mov al, %1
mov [DMA_CHANNEL], al 
mov eax, %2
mov [DMA_BUFFER_ADDRESS], eax
mov eax, %3
mov [DMA_BUFFER_LENGTH], eax
DMA_set_registers	[DMA_CHANNEL]	; Set register ports
DMA_set_mode		DMA_MODE_SINGLE, DMA_ADDRESS_INCREMENT, DMA_SINGLE_CYCLE, DMA_TRANSFER_WRITE
; DMA_MODE_BLOCK
; DMA_AUTO_INIT => sound card
cli					; Disable interrupts
DMA_status      	[DMA_STATE1]
;DMA_disable				; Disable DMA 8237A
DMA_disable_channel	[DMA_CHANNEL]	; Disable channel
DMA_mode		[DMA_MODE]	; Set DMA mode
DMA_clear		0		; Clear flip-flop (support 16-bit)
DMA_address		[DMA_BUFFER_ADDRESS] 	; set address
DMA_page		[DMA_BUFFER_ADDRESS] 	; set page
DMA_count		[DMA_BUFFER_LENGTH]	; set length (max. 65536)
DMA_enable_channel	[DMA_CHANNEL]	; Enable channel
DMA_enable				; Enable DMA 8237A
DMA_status      	[DMA_STATE2]
sti					; Enable Interrupts
%endmacro

; before
; 00000100
; after
; 00000000 ; channel 2 busy
;
;==============================================================================
; DMA 8237A (Direct Memory Access)
;==============================================================================
DMA_BUFFER_ADDRESS	DD	0		; 20-bit address
DMA_BUFFER_LENGTH	DD	0
DMA_CHANNEL		DB	0
DMA_MODE		DB	0
DMA_STATE1		DB	0
DMA_STATE2		DB	0
;;; SPECIFIC REGISTERS
DMA_ADDRESS_REGISTER	DB	0
DMA_COUNT_REGISTER	DB	0
DMA_PAGE_REGISTER	DB	0
;;; GENERIC REGISTERS
DMA_COMMAND_REGISTER	DB	0
DMA_REQUEST_REGISTER	DB	0
DMA_MASK_REGISTER	DB	0
DMA_MODE_REGISTER	DB	0
DMA_CLEAR_REGISTER	DB	0
;			ADDRESS	COUNT	PAGE
DMA_REGISTERS:
		DB	0x00,	0x01,	0x87		; CHANNEL 0
		DB	0x02,	0x03, 	0x83		; CHANNEL 1
		DB	0x04,	0x05,	0x81		; CHANNEL 2
		DB	0x06,	0x07,	0x82		; CHANNEL 3
		DB	0xC0,	0xC2,	0x8F		; CHANNEL 4
		DB	0xC4,	0xC6,	0x8B		; CHANNEL 5
		DB	0xC8,	0xCA,	0x89		; CHANNEL 6
		DB	0xCC,	0xCE,	0x8A		; CHANNEL 7

; Mode Selection (bits 7:6)
DMA_MODE_DEMAND		equ	0		; 00
DMA_MODE_SINGLE		equ	64		; 01
DMA_MODE_BLOCK		equ	128		; 10
DMA_MODE_CASCADE	equ	192		; 11

; Address Increment/Decrement (bit 5)
DMA_ADDRESS_INCREMENT	equ	0		; 0
DMA_ADDRESS_DECREMENT	equ	32		; 1

; AutoInitialization enable (bit 4)
DMA_SINGLE_CYCLE	equ	0		; 0
DMA_AUTO_INIT		equ	16		; 1

; Transfer Type (bits 3:2)
DMA_TRANSFER_VERIFY	equ	0		; 00
DMA_TRANSFER_WRITE	equ	4		; 01
DMA_TRANSFER_READ	equ	8		; 10

;=============================================================================
