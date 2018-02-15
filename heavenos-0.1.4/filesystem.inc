
; Read-Only FileSystem
; commands: "dir" and "cd <directory>"

%macro RAMDISK_read     0
mov esi, RAMDISK_ADDRESS
add esi, 36352 
mov edi, RAMDISK_BUFFER
%%next_copy:
mov eax, dword [es:esi]
mov dword [es:edi], eax
add esi, 4
add edi, 4
cmp edi, RAMDISK_BUFFER+41
jl %%next_copy
%endmacro

%macro FILESYSTEM_goto_root	0
mov esi, RAMDISK_ADDRESS+ROOTDIR_START
mov [DIRECTORY_POSITION], esi
%endmacro

%macro FILESYSTEM_read_dir_entry  1
mov esi, [DIRECTORY_POSITION] 
add esi, CLUSTER_SIZE
mov eax, %1
mov ebx, 4
mul ebx
add esi, eax 
mov eax, dword [es:esi]
mov dword %1, eax
; %1 = cluster number
; != 0xFFFFFFFF
%endmacro

%macro FILESYSTEM_read_file_header 1
mov edi, FILE_HEADER
mov esi, RAMDISK_ADDRESS+ROOTDIR_START
mov eax, %1
mov ebx, CLUSTER_SIZE
mul ebx
add esi, eax
;mov esi, RAMDISK_ADDRESS+ROOTDIR_START+4096
mov dword %1, esi 
%%next_header_byte:
mov al, byte [es:esi]
mov byte [es:edi], al
inc esi
inc edi
cmp edi, FILE_HEADER+FILE_HEADER_SIZE
jl %%next_header_byte
; %1 = file position
; information is on FILE_HEADER
%endmacro

%macro FILESYSTEM_change_dir	1
; %1 = address of the file name!
mov dword [FILESYSTEM_TEMP_VALUE], 0
%%next_dir:
FILESYSTEM_read_dir_entry	[FILESYSTEM_TEMP_VALUE]
FILESYSTEM_read_file_header	[FILESYSTEM_TEMP_VALUE]
inc dword [FILESYSTEM_TEMP_VALUE]
mov esi, FILE_HEADER
mov al, byte [es:esi]
and al, 0x01
cmp al, 0x01
jne %%next_dir

cmp_str FILE_HEADER+1, %1, [BOOLEAN_VALUE]

cmp byte [BOOLEAN_VALUE], 1
je %%dir_found
cmp dword [FILESYSTEM_TEMP_VALUE], 10
jl %%next_dir
%%dir_found:

%endmacro

%macro FILESYSTEM_attrib	2	
; %1 = number
; %3 = string address
mov edi, %2
mov esi, FILESYSTEM_ATTRIB_CHAR+7
mov eax, %1
mov ebx, 2
%%divide1:
xor edx, edx
div ebx
mov cl, byte '-'
cmp edx, 1
jne %%skip_attrib
mov cl, byte [ds:esi]
%%skip_attrib:
mov byte [ds:edi], cl
dec edi
dec esi
cmp eax, ebx
jge %%divide1
mov cl, byte '-'
cmp eax, 1
jne %%skip_attrib1
mov cl, byte [ds:esi]
%%skip_attrib1:
mov byte [es:edi], cl
cmp esi, FILESYSTEM_ATTRIB_CHAR
jne %%divide1
%endmacro


;============================================================================
RAMDISK_ADDRESS         equ     0x90000
; 0x100000
RAMDISK_BUFFER          DB '                                                           ',0
; times 512 0 for read files
FLOPPY_SIZE             equ     (512*18*2*80)   ; 1474560
CLUSTER_SIZE            equ     2048
FS_START                equ     512
FS_ENTRIES              equ     (FLOPPY_SIZE / CLUSTER_SIZE)    ; 720
ROOTDIR_START           equ     (FS_START+(4*FS_ENTRIES))+192   ; 3584
DIRECTORY_POSITION	DD	0
FILE_HEADER:
			DB	0		; attrib
	       times 35 DB 	0		; name (32 char length+endofline and stuff)
	       		DD	0		; length
FILE_HEADER_SIZE	equ $-FILE_HEADER
FILESYSTEM_TEMP_VALUE	DD	0
BOOLEAN_VALUE		DB	0
FILESYSTEM_ATTRIB_CHAR	DB '-------D'
;============================================================================


