
; Check VESA PM Info Block signature
%macro VESA_signature		1
mov edi, 0
mov esi, 32000
%%check_vesa:
mov eax, dword [es:edi]
inc edi
cmp eax, dword [VESA_STRING]
je %%end_check_vesa
cmp edi, [esi-3]
jne %%check_vesa
mov edi, 1
%%end_check_vesa:
dec edi
mov %1, edi
;;; TODO: checksum => (sum of all bytes / checksum) = 0 ?
%endmacro

;============================================================================
VESA_STRING		DD 'PMID'
VESA_PM_INFOBLOCK:
			DD 'PMID'	; PM Info Block Signature
			DW 0		; Offset of PM entry point within BIOS
			DW 0		; Offset of PM initialization entry point
			DW 0		; Selector to BIOS data area emulation block
			DW 0xA000	; Selector to access 0xA0000 physical mem
			DW 0xB000	; Selector to access 0xB0000 physical mem
			DW 0xB800	; Selector to access 0xB8000 physical mem
			DW 0xC000	; Selector to access code segment as data
			DB 0		; Set to 1 when in protected mode
			DB 0		; Checksum byte for structure
;==============================================================================

; VESA (VBE SPECIFICATION) FUNCTIONS
;
; 'PMID' vesa signature
; scan the first 32KB of BIOS image
; make a FAR CALL to the ENTRY POINT

