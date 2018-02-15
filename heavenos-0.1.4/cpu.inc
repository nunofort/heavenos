
; Detect CPU Type (only processor family)
%macro CPU_family		1
  pushfd                  ;At least an 80386. Save flags twice:
  pushfd                  ;first for later restoration,
  pop     eax             ;second to get them
  or      eax,240000h     ;set ID and AC flags
  push    eax
  popfd                   ;tell flag register about it
  pushfd                  ;did the CPU take the bait?
  pop     eax
  shr     eax,10h
  and     al,24h          ;isolate ID and AC bits
  jz      %%_3            ;can't set either flag ===>
                          ;must be an 80386
  cmp     al,4            ;can't set ID bit?
  jz      %%_4            ;no: must be an 80486
  mov     ax,1            ;Pentium or Pentium Pro
  CPUID                   ;get CPU ID
  mov     al,ah           ;forget about model and stepping
                          ;numbers
  and     al,0fh          ;isolate ID
  jmp     short %%_4
%%_3:
  mov     al,3            ;80386
%%_4:
  popfd                   ;restore flags
  mov byte [%1], al 
%endmacro

%macro CPU_check_cpuid	1
mov dword  [%1], 0
cli
pushfd      		; Save EFLAGS to stack
pop eax    		; Store EFLAGS in EAX
mov ebx, eax    	; Save in EBX for testing later
xor eax, 00200000h   	; Switch bit 21
push eax    		; Copy changed value to stack
popfd    		; Save changed EAX to EFLAGS
pushfd    		; Push EFLAGS to top of stack
pop eax    		; Store EFLAGS in EAX
cmp eax, ebx    	; See if bit 21 has changed
jz %%no_cpuid  		; If no change, no CPUID
mov dword [%1], 1
%%no_cpuid:
sti
%endmacro

; i586+ Processors only
; return 'GenuineIntel' OR 'AuthenticAMD'
CPU_signature:
mov edi, CPU_SIGNATURE 
xor eax, eax    ; eax = 0 (level 0), EBX:EDX:ECX have CPU SIGNATURE on return
CPUID		
mov dword [es:edi+0], ebx
mov dword [es:edi+4], edx
mov dword [es:edi+8], ecx
retf

; i586+ Processor only
CPU_id:
xor eax, eax		; eax = 0 (level 0)
CPUID			; return EAX = maximum available level
cmp eax, 0
je exit_id
mov eax, 1		; level 1
CPUID			; return EAX
			; Processor Type(INTEL only)
			; bits 13..12 = 0 - normal
			;               1 - overdrive system
			;               2 - dual system
			; bits 11..8 = Family
			; bits 7..4 = Model
			; bits 3..0 = stepping/revision
			; return EDX = processor feature flags
			; bit 23 = MMX support!
mov dword [CPU_ID], eax
mov dword [CPU_FLAGS], edx
exit_id:
retf

; i586+ Processor only
%macro CPU_extended_id		1
mov eax, 0x80000000	; first extended level
CPUID			; return EAX = maximum available extended level
mov %1, eax
; extended levels return information specific to the processor
; 3DNow support or Processor Name !
%endmacro

%macro CPU_detect		1
; %1 = return STEPPING
mov esi, CPU_UNKNOWN_STRING
mov dword [CPU_STRING], esi 
mov esi, CPU_AMD_SIGNATURE
mov edi, CPU_SIGNATURE
%%next_amd:
mov ebx, dword [ds:esi]
mov eax, dword [ds:edi]
add esi, 4
add edi, 4
cmp eax, ebx
jne %%skip_amd
cmp edi, CPU_SIGNATURE+12
jne %%next_amd
mov esi, CPU_AMD_ID
jmp short %%cpu_find
%%skip_amd:
mov esi, CPU_INTEL_SIGNATURE
mov edi, CPU_SIGNATURE
%%next_intel:
mov ebx, dword [ds:esi]
mov eax, dword [ds:edi]
add esi, 4
add edi, 4
cmp eax, ebx
jne %%cpu_end_detect
cmp edi, CPU_SIGNATURE+12
jne %%next_intel
mov esi, CPU_INTEL_ID
%%cpu_find
mov ebx, dword [CPU_ID]
mov al, 00001111b
and al, bl
mov byte [%1], al
shr ebx, 4
%%next_cpu:
mov ax, word [ds:esi]
add esi,33 
cmp ax, bx
je %%cpu_found 
cmp word [ds:esi], 0xFFFF
je %%cpu_end_detect
jmp short %%next_cpu
%%cpu_found:
sub esi, 31
mov dword [CPU_STRING], esi 
%%cpu_end_detect:
%endmacro

;===========================================================================;
; CPU ID (low 16 bits of EAX)                                               ;
;===========================================================================;
CPU_STRING        	DD      0x00000000 ; COOL Pointer!!! :)
CPU_SIGNATURE		DB '            ',0
CPU_AMD_SIGNATURE	DB 'AuthenticAMD'
CPU_INTEL_SIGNATURE	DB 'GenuineIntel'
CPU_ID			DD 0
CPU_FLAGS		DD 0
CPU_UNKNOWN_STRING	DB 'Unknown Processor Description',0
; TODO : colocar info sobre cpu's 386 e 486
; guardar o valor do EDX em memoria depois do reset
;---------------------------------------------------------------------------;
; AMD                                                                       ;
;---------------------------------------------------------------------------;
CPU_AMD_ID:
				DW 0000000001000000b
				DB 'Am486 or Am5x86 Processor     ',0
				DW 0000000001000001b
				DB 'Am486 or Am5x86 Processor     ',0
				DW 0000000001010000b
				DB 'AMD-K5(tm) Processor (Model 0)',0
				DW 0000000001010001b
				DB 'AMD-K5 Processor (Model 1)    ',0
				DW 0000000001010010b
				DB 'AMD-K5 Processor (Model 2)    ',0
				DW 0000000001010011b
				DB 'AMD-K5 Processor (Model 3)    ',0
				DW 0000000001010110b
				DB 'AMD-K6 Processor (Model 6)    ',0
				DW 0000000001010111b
				DB 'AMD-K6 Processor (Model 7)    ',0
				DW 0000000001011000b
				DB 'AMD-K6-2 Processor (Model 8)  ',0
				DW 0000000001011001b
				DB 'AMD-K6-III Processor (Model 9)',0
				DW 0000000001100001b
				DB 'AMD Athlon(tm) Model 1        ',0
				DW 0000000001100010b
				DB 'AMD Athlon(tm) Model 2        ',0
				DW 0000000001100100b
				DB 'AMD Athlon(tm) Model 4        ',0
				DW 0000000001100011b
				DB 'AMD Duron(tm) Processor       ',0
				DW 0xFFFF
;---------------------------------------------------------------------------;
; INTEL                                                                     ;
;---------------------------------------------------------------------------;
; RESERVED(2 bits) TYPE(2 bits) FAMILY(4 bits) MODEL(4 bits) STEPPING(4 bits)
CPU_INTEL_ID:
				DW 0000000001000000b
				DB 'Intel486(tm) DX Processor     ',0
				DW 0000000001000001b
				DB 'Intel486(tm) DX Processor     ',0
				DW 0000000001000010b
				DB 'Intel486 SX Processor         ',0
				DW 0000000001000011b
				DB 'Intel486 DX2(tm) Processor    ',0
				DW 0000000001000101b
				DB 'Intel486 SX2(tm) Processor    ',0
				DW 0000000001000100b
				DB 'Intel486 SL Processor         ',0
				DW 0000000001000111b
				DB 'IntelDX2 Write-Back Processor ',0
				DW 0000000001001000b
				DB 'IntelDX4(tm) Processor        ',0
				DW 0000000101001000b
				DB 'IntelDX4 OverDrive Processor  ',0
				DW 0000000001010001b
				DB 'Pentium 60/66 Mhz             ',0
				DW 0000000001010010b
				DB 'P75/90/100/120/133/166/200 Mhz',0
				DW 0000000101010001b
				DB 'Pentium OverDrive 60/66 Mhz   ',0
				DW 0000000101010010b
				DB 'P 75/90/100/120/133 OverDrive ',0
				DW 0000000101010011b
				DB 'Pentium OverDrive for Intel486',0
				DW 0000000001010100b
				DB 'Pentium 166/200 Mhz MMX(tm)   ',0
				DW 0000000101010100b
				DB 'P 75/90/100/120/133 OD MMX(tm)',0
				DW 0000000001100001b
				DB 'Pentium Pro Processor         ',0
				DW 0000000001100011b
				DB 'Pentium II Processor, model 3 ',0
				DW 0000000001100101b
				DB 'Pentium II Processor, model 5 ',0
				DW 0000000001100110b
				DB 'Pentium Celeron, model 6      ',0
				DW 0000000001100111b
				DB 'Pentium III Processor, model 7',0
				DW 0000000001101000b
				DB 'Pentium III Processor, model 8',0
				DW 0000000001101010b
				DB 'Pentium III Xeon, model A     ',0
				DW 0000000101100011b
				DB 'Pentium II OverDrive Processor',0
				DW 0000000011110000b
				DB 'Intel Pentium 4 Processor     ',0
				DW 0xFFFF


