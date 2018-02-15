;
; By Nuno Miguel (codingheaven@gmail.com)
;
[BITS 16]
[ORG 0x7C00]

; DOS header alike boot sector 
;==========================================================
;DB 0xEB,0x1E,0x90 ; SHORT JUMP to 0x1E
;DB 'HeavenOS'     ; OEM Name and Number
;DW 512		  ; Bytes per Sector (512)
;DB 0	 	  ; Sectors per Allocation Unit (cluster)
;DW 0		  ; Reserved seectors (for Boot Record)
;DB 0		  ; Number of FATs
;DW 0		  ; Number of Root Directories Entries
;DW 0		  ; Number of logical Sectors
;DB 0xF0           ; Medium Descriptor Byte (0xF0 - FD, 0xF8 - HD)   
;DW 0		  ; Sectors per FAT
;DW 18		  ; Sectors per TRACK
;DW 2		  ; Number of Heads
;DW 0		  ; Number of Hidden Sectors
;DW 0xAA55	  ; Load Signature - 0xAA55	
;==========================================================
; Boot Loader
;==========================================================

; save EDX value in memory (the last 4 bytes of conventional memory)
; CPU ID Information
mov ax, 0x9000
mov es, ax
mov di, 0xFFFC
mov dword [es:di], edx

; setting regs
mov ax, 0 
mov ds, ax
mov es, ax
mov ss, ax
mov sp, (0x7C00 + 0x400)
mov bp, 0 

jmp  0:start

%macro print_number     3
mov ax, 0xB800
mov es, ax
mov di, 160*%1 + 2*(%2+5)
mov ax, %3
mov bx, 10
%%next_digit:
div bx
mov dh, 0x03
add dl, 48
mov word [es:di], dx
times 2 dec di
cmp ax, 10
jge %%next_digit
mov word [es:di], ax
%endmacro

; COLD REBOOT
cold_boot:
mov             ax, 0x0040                      ; ax = 0040h
mov             es, ax                          ; point es to ax
mov             word [es:0072h], 0x0000         ; Reset
db              0xEA                            ; JUMP
dw              0x0000                          ; FFFF:0000
dw              0xFFFF                          ; ! REBOOTING !
;int     0x19                                   ; warm boot pc 

start:

;=======================================================================
; Check if CPU is 386+
;=======================================================================
pushf                   ; save flags for later
xor     ah,ah           ; clear high byte
push    ax              ; push AX on the stack
popf                    ; pop this value into the flag register
pushf                   ; push flags on the stack
pop     ax              ; ...and get flags into AX
and     ah,0f0h         ; try to set the high nibble
cmp     ah,0f0h         ; on a 80386, the high nibble can never be 0f0h
je      cold_boot
mov     ah,70h          ; now try to set NT and IOPL
push    ax
popf
pushf
pop     ax
and     ah,70h          ; if they couldn't be modified, no 386 is installed
jz      cold_boot
popf                    ; restore flags
;=======================================================================

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

;;; RM_Write_Boot_Mensage
mov di, 160*0x01 + 2*0x01
mov si, boot_msg
mov ah, 0x03
next_char:
mov al, byte [si]
mov word [es:di], ax
times 2 inc di
inc si
cmp byte [si],0
jne next_char


;=======================================================================
; Checking Kernel Image Length
;=======================================================================
mov    ax, 0x0050
mov    es, ax
mov    bx, 0x0000
mov    ah, 0x02
mov    dl, 0x00
mov    dh, HEADER_HEAD
mov    ch, HEADER_TRACK
mov    cl, HEADER_SECTOR
mov    al, 19
sub    al, cl
int    0x13
mov    ax, word [es:bx+36]
mov    [KERNEL_SIZE], ax
;=======================================================================

;mov    word [KERNEL_SIZE], 5000
;cmp     ax, 28230
;jne    skip_msg

;mov ax, 0xB800
;mov es, ax
;mov di, 160*0x03 + 2*0x01
;mov si, boot_msg
;mov ah, 0x03
;next_char1:
;mov al, byte [si]
;mov word [es:di], ax
;times 2 inc di
;inc si
;cmp byte [si],0
;jne next_char1
;skip_msg:


;=======================================================================
; Copying Kernel Image from Floppy Filesystem to Memory
; point es:bx to 0x0050:0x0000
;=======================================================================
mov    ax, 0x0050  		; segment
mov    es, ax          
mov    bx, 0x0000  		; offset 
; load data from disk to [es:bx]
mov    ah, 0x02    		; read sector(s)
mov    dl, 0x00    		; disk (0x00 - First FD, 0x80 - First HD)
mov    dh, KERNEL_HEAD       	; head  
mov    ch, KERNEL_TRACK       	; track (0-1023)
mov    cl, KERNEL_SECTOR      	; sector (2 higher bits = 10-9 bits of track)

mov    al, 19	   		; max. sectors = 18 
sub    al, cl	   		; n sectors
mov    [ADDRESS_POSITION], bx
pusha
int    0x13
xor    ah, ah 
mov    bx, 512
mul    bx
add    [ADDRESS_POSITION], ax
next_head:
popa
inc    dh
cmp    dh, 0x02    		; head < 2 ?
jne     skip_head
mov    dh, 0			; head 0
inc    ch   			; next track
skip_head:
mov    ax, 0x0050  		; segment
mov    es, ax
mov    bx, [ADDRESS_POSITION]
mov    ah, 0x02    		; read sector(s)
mov    dl, 0x00    		; disk type 
mov    cl, 0x01       		; sector 1
mov    al, 18
pusha
int    0x13
cmp    ah, 0
je     skip_reset
do_reset:
;mov    ax, 0
;delay:
;inc    ax
;cmp    ax, 0x100		; some delay
;jne    delay
mov    ah, 0x00    		; reset drive
mov    dl, 0x00    		; disk type 
xor    ax, ax
int    0x13
;cmp    ah, 0
;jne    do_reset	
jmp    0:next_head			; read again the head
skip_reset:
add    [ADDRESS_POSITION], word (512*18)
cmp    bx, [KERNEL_SIZE]
jg     end_copy 
print_number 	3,2,[ADDRESS_POSITION]
print_number 	4,2,[KERNEL_SIZE]
jmp 0:next_head
end_copy:
;====================================================================

; TODO : Read a Sector for Protection in the future
; Password protection ?
;====================================================================

mov ax, 0xB800
mov es, ax
;;; RM_Write_Boot_Mensage
mov di, 160*0x02 + 2*0x01
mov si, boot_msg
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
cmp  al, 0x01     	; check ESC key           
jne  check_keyboard          
;mov ah,0	; await key pressed
;int 16h

; Disable Floppy Motor
;mov al,0001100b  ;Disable all motors, controller enabled, DMA  & IRQ enabled
;mov dx,0x3F2     ;Digital Output Register
;out dx,al        ;Sending the command to the controller

; turn off floppy motor
mov dx,3F2h
xor al,al
out dx,al

; JUMP to address 0050:0000 RM (0x00000500 PM)
jmp  0x0050:0


;=======================================================================
boot_msg		DB 	'Loading Boot Sector...',0 
FLOPPY_SIZE		equ	(512*18*2*80)	; 1474560
CLUSTER_SIZE		equ	2048
FS_START		equ	512
FS_ENTRIES              equ	(FLOPPY_SIZE / CLUSTER_SIZE)	; 720
ROOTDIR_START		equ	(FS_START+(4*FS_ENTRIES))+192	; 3584
HEADER_POSITION		equ	ROOTDIR_START+CLUSTER_SIZE+(4*512) ; 7680
HEADER_HEAD             equ     (((HEADER_POSITION/512) / 18) % 2)
HEADER_TRACK            equ     ((HEADER_POSITION/512) / (18*2))
HEADER_SECTOR		equ	((HEADER_POSITION/512) % 18) + 1
KERNEL_POSITION		equ	ROOTDIR_START+CLUSTER_SIZE+(4*512)+CLUSTER_SIZE
KERNEL_HEAD		equ 	(((KERNEL_POSITION/512) / 18) % 2)
KERNEL_TRACK		equ	((KERNEL_POSITION/512) / (18*2))
KERNEL_SECTOR		equ	((KERNEL_POSITION/512) % 18) + 1 
KERNEL_SIZE             DW      0
SECTOR			DB 	0
HEAD			DB	0
TRACK			DB	0
ADDRESS_POSITION	DW	0
NUM_SECTORS		DB	0

;S = (lsn mod spt) + 1
;H = (lsn / spt) mod nh
;T = lsn / (spt*nh)
;
;S is the physical sector number
;H is the head number
;T is the track number
;lsn is the logical sector number
;spt is the number of sectors per track
;nh is the number of heads 
;/ is an integer division
;mod is the modulo operation

;-----------------------------------------------------------------------
times 510-($-$$) DB 0
DW	0xAA55 		; BOOT LOADER SIGNATURE
;-----------------------------------------------------------------------

; PM Address(REAL) = 16 * RM Segment + RM Offset
;  Memory Address   
;     65536        = 16 * 4096 + 0 
;                  = (RM Segment << 4) + RM Offset
;
; 16*0x1000 = 65536*1 start first segment 
; 16*0x2000 = 65536*2 start second segment
; 16*0x3000 = 65536*3 start third segment
