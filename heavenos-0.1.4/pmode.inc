switch_to_pmode: 
        mov     eax, cr0                ; Get Control Register 0
        or      eax, 1                  ; set PE bit (bit 0) in (e)ax
        mov     cr0, eax                ; Switch to Protected Mode
ret

switch_to_rmode:  
        mov     eax, cr0
        and     al, 0FEh                ;
        mov     cr0, eax                ; Switch to Real Mode
ret

%macro  load_program_at         2
        ; rmode macro
        db              0xEA            ; Machine Language to Far Jump to
        dd              %2              ; 32-bit offset address
        dw              %1              ; 16-bit segment address
%endmacro

; ESC   ; gives the next instruction for the co-processor
; HLT   ; halt the processor - stop the processor


