
;=======================================================================
; Checking Kernel Image Length
;=======================================================================
;mov    ax, 0x0050
;mov    es, ax
;mov    bx, 0x0000
;mov    ah, 0x02
;mov    dl, 0x00
;mov    dh, HEADER_HEAD
;mov    ch, HEADER_TRACK
;mov    cl, HEADER_SECTOR
;mov    al, 19
;sub    al, cl
;int    0x13
;mov    ax, word [es:bx+36]
;mov    [KERNEL_SIZE], ax
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

mov    ax, 0x0050               ; segment
mov    es, ax
mov    bx, 0x0000               ; offset 
; load data from disk to [es:bx]
mov    ah, 0x02                 ; read sector(s)
mov    dl, 0x00                 ; disk (0x00 - First FD, 0x80 - First HD)
mov    dh, KERNEL_HEAD          ; head  
mov    ch, KERNEL_TRACK         ; track (0-1023)
mov    cl, KERNEL_SECTOR        ; sector (2 higher bits = 10-9 bits of track)

mov    al, 19                   ; max. sectors = 18 
sub    al, cl                   ; n sectors
;pusha
int    0x13
;popa
;push   bx
;mov    ah, 0
;mov    bx, 512
;mul    bx
;pop    bx
;add    bx, ax
;next_head:
;inc    dh

;cmp    dh, 0x02                 ; head < 2 ?
;jne     skip_head
;mov    dh, 0                    ; head 0
;inc    ch                       ; next track
;skip_head:
;mov    ax, 0x0050               ; segment
;mov    es, ax
;mov    ah, 0x02                 ; read sector(s)
;mov    dl, 0x00                 ; disk type 
;mov    cl, 0x01                 ; sector 1
;mov    al, 18
;pusha
;int    0x13
;popa
;cmp    ah, 0
;je     skip_reset
;do_reset:
;mov    ah, 0x00                 ; reset drive
;mov    dl, 0x00                 ; disk type 
;xor    ax, ax
;pusha
;int    0x13
;popa
;cmp    ah, 0
;jne    skip_head
;skip_reset:
;add    bx, 512*18
;cmp    word bx, [KERNEL_SIZE]
;jl     next_head

