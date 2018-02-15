
%macro number_to_text	3	
; %1 = number
; %2 = base N (if N=16(div 16) do 4 shift's to right, for speedup this)
; %3 = string address
mov edi, %3
mov esi, UTIL_HEX_DIGITS
mov eax, %1
mov ebx, %2
%%divide:
xor edx, edx
div ebx
mov cl, byte [ds:esi+edx]
mov byte [ds:edi], cl
dec edi
cmp eax, ebx
jge %%divide
cmp eax, 0
je %%end
mov cl, byte [ds:esi+eax]
mov byte [es:edi], cl
%%end:
%endmacro

%macro copy_str		2	
mov esi, %1
mov edi, %2
%%next_byte:
mov al, byte [es:esi]
mov byte [es:edi], al
inc esi
inc edi
cmp byte [es:esi], 0
jne %%next_byte
%endmacro

%macro cmp_str       3
; %1 = Test String
; %2 = Right String
; %3 = 0(false) or 1(true)
mov dword %3, 0
mov edi, %1
mov esi, %2
%%next_cmp:
mov al, byte [ds:edi]
mov bl, byte [es:esi]
cmp al, bl
jne %%no_string
inc edi
inc esi
cmp al, 0
jne %%next_cmp
mov dword %3, 1
%%no_string    
%endmacro

%macro clean_str	1
mov edi, %1
mov al, 32
%%next_clean:
mov byte [es:edi], al
inc edi
cmp byte [es:edi], 0
jne %%next_clean
%endmacro

%macro get_bit		2
mov eax, %1
mov ebx, %2
shr eax, ebx
and eax, 1
mov %1, eax
%endmacro

UTIL_HEX_DIGITS			DB '0123456789ABCDEF'
; char(48+digit)
