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


;%include "copy_kernel.inc"

;=======================================================================
; Copying Kernel Image from Floppy Filesystem to Memory
; point es:bx to 0x0050:0x0000
;=======================================================================
;INT 13 - DISK - READ SECTOR(S) INTO MEMORY
;	AH = 02h
;	AL = number of sectors to read (must be nonzero)
;	CH = low eight bits of cylinder number
;	CL = sector number 1-63 (bits 0-5)
;	     high two bits of cylinder (bits 6-7, hard disk only)
;	DH = head number
;	DL = drive number (bit 7 set for hard disk)
;	ES:BX -> data buffer
;Return: CF set on error
;	    if AH = 11h (corrected ECC error), AL = burst length
;	CF clear if successful
;	AH = status (see #00234)
;	AL = number of sectors transferred (only valid if CF set for some
;	      BIOSes)
;Notes:	errors on a floppy may be due to the motor failing to spin up quickly
;	  enough; the read should be retried at least three times, resetting
;	  the disk with AH=00h between attempts
;	most BIOSes support "multitrack" reads, where the value in AL
;	  exceeds the number of sectors remaining on the track, in which
;	  case any additional sectors are read beginning at sector 1 on
;	  the following head in the same cylinder; the MSDOS CONFIG.SYS command
;	  MULTITRACK (or the Novell DOS DEBLOCK=) can be used to force DOS to
;	  split disk accesses which would wrap across a track boundary into two
;	  separate calls
;	the IBM AT BIOS and many other BIOSes use only the low four bits of
;	  DH (head number) since the WD-1003 controller which is the standard
;	  AT controller (and the controller that IDE emulates) only supports
;	  16 heads
;	AWARD AT BIOS and AMI 386sx BIOS have been extended to handle more
;	  than 1024 cylinders by placing bits 10 and 11 of the cylinder number
;	  into bits 6 and 7 of DH
;	under Windows95, a volume must be locked (see INT 21/AX=440Dh/CX=084Bh)
;	  in order to perform direct accesses such as INT 13h reads and writes
;	all versions of MS-DOS (including v7 [Win95]) have a bug which prevents
;	  booting on hard disks with 256 heads, so many modern BIOSes provide
;	  mappings with at most 255 heads
;
	  

mov   word [COPY_POSITION], 0x0000
mov     byte [SECTOR], 2
mov     byte [HEAD], 0 
mov     byte [TRACK], 0 
mov	    byte [NUM_SECTORS], 17
mov   word [COPY_INC], 512*17

next_head:
mov    ax, 0x0050               ; Real Mode Memory Segment
mov    es, ax
mov    bx, [COPY_POSITION]      ; offset 
; load data from disk to [es:bx]
mov    ah, 0x02              ; read sector(s)
mov    dl, 0x00               ; disk (0x00 - First FD, 0x80 - First HD)
mov    dh, [HEAD]           ; head  
mov    ch, [TRACK]          ; track (0-1023)
mov    cl, [SECTOR]	      ; sector (2 higher bits = 10-9 bit of track)
mov    al, [NUM_SECTORS]   ; max. sectors = 18 
int    0x13
;cmp ah, 0			; Operation was successfull
;je readok
;mov    ah, 0x00                 ; reset drive
;mov    dl, 0x00                 ; disk type 
;;xor    ax, ax
;int    0x13
;jmp short next_head
;readok:
inc    byte [HEAD]		;dh
cmp  byte [HEAD], 0x02 
jne    skip_head
mov	byte [HEAD], 0x00
inc    byte [TRACK]		;ch
skip_head: 
mov byte [NUM_SECTORS], 18
mov byte [SECTOR], 1
mov ax, word [COPY_INC]
add word [COPY_POSITION], ax 
mov word [COPY_INC], 512*18
cmp    byte [TRACK], 0x02
jne    next_head

;   * For `MUL r/m8', `AL' is multiplied by the given operand; the
;     product is stored in `AX'.

;   * For `MUL r/m16', `AX' is multiplied by the given operand; the
;     product is stored in `DX:AX'.



;====================================================================
; TODO : Read a Sector for OS Signature ?
;====================================================================

mov ax, 0xB800
mov es, ax
mov di, 160*0x02 + 2*0x01
mov si, jump_msg
mov ah, 0x03
next_charx:
mov al, byte [si]
mov word [es:di], ax
times 2 inc di
inc si
cmp byte [si],0
jne next_charx

check_keyboard:
in   al, 0x60                
cmp  al, 0x01     	; check ESC key           
jne  check_keyboard          
;mov ah,0	; await key pressed
;int 16h

; Disable Floppy Motor
mov al,0001100b  ;Disable all motors, controller enabled, DMA  & IRQ enabled
mov dx,0x3F2     ;Digital Output Register
out dx,al        ;Sending the command to the controller

; turn off floppy motor
;mov dx,3F2h
;xor al,al
;out dx,al

; JUMP to address 0050:0000 RM (0x00000500 PM)
jmp  0x0050:0


;=======================================================================
boot_msg		DB 	'Loading Boot Sector...',0 
jump_msg		DB	'Press ESC for Jumping to Kernel PM Code...',0
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
;KERNEL_HEAD		equ 	(((KERNEL_POSITION/512) / 18) % 2)
;KERNEL_TRACK		equ	((KERNEL_POSITION/512) / (18*2))
;KERNEL_SECTOR		equ	((KERNEL_POSITION/512) % 18) + 1 
KERNEL_HEAD		equ 	0x00	
KERNEL_TRACK		equ	0x00	
KERNEL_SECTOR		equ	0x02	
KERNEL_SIZE             DW      0
SECTOR			DB 	0
HEAD			DB	0
TRACK			DB	0
ADDRESS_POSITION	DW	0
NUM_SECTORS		DB	0
COPY_POSITION		DW	0
COPY_INC			DW   0

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
