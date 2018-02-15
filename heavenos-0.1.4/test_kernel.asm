[BITS 16]
[ORG 0x00000500]

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

;;; RM_Write_Message
mov ax, 0xB800
mov es, ax
;; RM_Write_Boot_Mensage
mov di, 160*0x02 + 2*0x01
mov si, kernel_msg
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



kernel_msg	 	DB 'Loading HeavenOS Kernel V0.1.4',0

