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
; Copying Kernel Image from Floppy Filesystem to Memory
; point es:bx to 0x0050:0x0000
;=======================================================================
mov    ax, 0x0050  		; segment
mov    es, ax          
mov    bx, 0x0000  		; offset 
; load data from disk to [es:bx]
mov    ah, 0x02    		; read sector(s)
mov    dl, 0x00    		; disk (0x00 - First FD, 0x80 - First HD)
mov    dh, 0       		; head  
mov    ch, 0       		; track (0-1023)
mov    cl, 2      			; sector (2 higher bits = 10-9 bits of track)
mov    al, 17	   		; max. sectors = 18 

int  	0x13			;READ

;second pass
add	bx, 512*17		; next offset
mov dh, 1			; another head  
mov ch, 0			; track 0 (0-1023)

next_sectors:
mov    ax, 0x0050  		; segment
mov    es, ax          
;mov    bx, 0x0000  		; offset 
; load data from disk to [es:bx]
mov    ah, 0x02    		; read sector(s)
mov    dl, 0x00    		; disk (0x00 - First FD, 0x80 - First HD)
mov    cl, 1      			; sector (2 higher bits = 10-9 bit of track)
mov    al, 18	   		; max. sectors = 18  (was 18)

int    0x13

jmp read_out 			; skip the following code it has read enough for 12k
; temp skip this
inc	dh
add	bx, 512*18
cmp dh, 2
jne next_head
mov	dh, 0
inc ch
next_head:
cmp ch, 2
jne next_sectors 

;cmp    ah, 0
;je     skip_reset
;do_reset:
;mov    ah, 0x00    		; reset drive
;mov    dl, 0x00    		; disk type 
;xor    ax, ax
;int    0x13
;jmp short copy_kernel
;skip_reset:

read_out:
;====================================================================
; TODO : Read a Sector for Protection in the future
; Password protection ?
;====================================================================

mov ax, 0xB800
mov es, ax
;;; RM_Write_Boot_Mensage
mov di, 160*0x02 + 2*0x01
mov si, jump_msg
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

; Disable Floppy Motor
;mov al,0001100b  ;Disable all motors, controller enabled, DMA  & IRQ enabled
;mov dx,0x3F2     ;Digital Output Register
;out dx,al        ;Sending the command to the controller

; turn off floppy motor
;mov dx,3F2h
;xor al,al
;out dx,al

; JUMP to address 0050:0000 RM (0x00000500 PM)
jmp  0x0050:0


;=======================================================================
boot_msg		DB 	'Loading Boot Sector...',0 
jump_msg		DB	'Press ESC for Jumping to Kernel 32-bit PM Code...',0

debug_msg		DB	'Here',0

Yval			DW	2   ; current Y console position

;============================
; added by stdio	START
;============================

print_d:
	push es			; Save es register
	pusha			; save general purpose registers (E)AX, (E)CX, (E)DX, (E)BX, (E)SP, (E)BP, (E)SI, (E)DI
	; increment Line on print
	mov si,Yval		; Address of Yval
	mov di, word [si]	; copy contents pointed by si (Yval)
	add di,1		; sum + 1
	mov word [si] ,di	; store in mem pointed by si (Yval)
	imul di,160 		; Y*columns (160)
	add di,2 		; X value

	; copy char to video , text mode	
	;mov di, 160*2 + 2*0x01
	mov ax,0xB800		; Address of text memory
	mov es,ax		; Make es, 0xB800
	mov si, debug_msg	; si is address of debug_msg
	mov ah, 0x04		; color/attribute
	pd_next_char:
	mov al, byte [si]	; char
	mov word [es:di], ax	; copy char to video mem
	times 2 inc di		; increase by 2 video
	inc si			; point to next char
	cmp byte [si],0		; is end of string?
	jne pd_next_char	; if not go to next char
	popa			; restore general purpose registers
	pop es			; restore es
ret
;======================== END

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
times 510-($-$$) DB 0x0 ; stdio, nop operation
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
