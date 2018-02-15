
%macro MEMORY_get_ram		1
mov ebx, 0x00100000 ; address 1 048 576
%%next_byte:
mov al, 0xFF
mov byte [es:ebx], 0x00
mov al, byte [es:ebx]
inc ebx
cmp al, 0x00
je %%next_byte
dec ebx
xor edx, edx
mov eax, ebx
mov ebx, 0x00100000
div ebx
mov %1, eax
%endmacro

; cada programa pede ao kernel a memoria(bytes) q precisa que seja alocada
; e o kernel devolve-lhe um ponteiro para esse bloco de memoria

