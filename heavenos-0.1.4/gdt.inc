
; Segment Selector Struc
;
;15-bit                                 2-bit   0-bit
;=================================================
;             index	               | TI | RPL|
;=================================================
; index : index into GDT or LDT
; TI	: Table indicator	0=GDT	1=LDT
; RPL   : Requested Privilege Level	00=0	01=1	10=2	11=3
;  

; Segment Descriptor Format
;
;31-bit          23-bit		          15-bit                            0
;============================================================================
;  Base 31..24  |G|DB|res|av|Limit 19..16|P| DPL |DT| Type | Base 23..16  
;----------------------------------------------------------------------------
;              Base 15..0	         |	    Limit 15..0          
;============================================================================
; Base 31..24, Base 23..16, Base 15..0 : Segment Base
; Limit 19..16, Limit 15..0 : Segment Limit
;    G : Granularity	0=Byte Granularity		1=Page Granularity
;   DB : Segment Size 	0=16-bit addresses(default)	1=32-bit addresses(Big)
;  res : Reserved 
;   av : Available for Operating System
;    P : Segment Present	0=Segment Not in Memory	1=Segment in Memory
;  DPL : Descriptor Privilege Level	00=0	01=1	10=2	11=3
;   DT : Descriptor Type	0=System Segment	1=Application Segment
; Type : Type of System or Application Segment
;

; For DT=1
;===================================================== 
;  Type
;=====================================================
;  0000		read only
;  0001		read only, accessed
;  0010		read/write
;  0011		read/write, accessed
;  0100		read only, expand down
;  0101		read only, expand down, accessed
;  0110		read/write, expand down
;  0111		read/write, expand down, accessed
;  1000		execute only
;  1001		execute only, accessed
;  1010		execute/read
;  1011		execute/read, accessed
;  1100		execute only, conforming
;  1101		execute only, conforming, accessed
;  1110		execute/read, conforming
;  1111		execute/read, conforming, accessed
;
; For DT=0
;======================================================
;  Type
;======================================================
;  0000		reserved
;  0001		available 80286 TSS
;  0010		LDT
;  0011		active 80286 TSS
;  0100		80286 Call Gate
;  0101		Task Gate (80286+i386)
;  0110		80286 Interrupt Gate
;  0111		80286 Trap Gate
;  1000		reserved
;  1001		available i386 TSS
;  1010		reserved
;  1011		active i386 TSS
;  1100		i386 Call Gate
;  1101		reserved
;  1110		i386 Interrupt Gate
;  1111		i386 Trap Gate 
;	 
;=======================================================================
gdt_ptr:
	  dw	gdt_size-1	; Length of GDT
	  dd	gdt		; GDT address

gdt:
null_des  times	8 DB 0		; NULL descriptor
; selector 0x0008
; execute/read, accessed
code32:   
	  dw	0xFFFF		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10011011b	; P=1 DPL=00 DT=1 TYPE=1011
	  db	11001111b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]
; selector 0x0010
; read/write, accessed
data32:   
	  dw	0xFFFF		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10010011b	; P=1 DPL=00 DT=1 TYPE=0011
	  db	11001111b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]
; selector 0x0018
int_gate: 
	  dw	0xFFFF		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10001110b	; P=1 DPL=00 DT=0 TYPE=1110
	  db	11001111b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]
; selector 0x0020
call_gate: 
	  dw	0xFFFF		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10001100b	; P=1 DPL=00 DT=0 TYPE=1100
	  db	11001111b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]
; selector 0x0028
active_tss: 
	  dw	0xFFFF		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10001011b	; P=1 DPL=00 DT=0 TYPE=1011
	  db	11001111b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]
; selector 0x0030
avail_tss: 
	  dw	0xFFFF		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10001001b	; P=1 DPL=00 DT=0 TYPE=1001
	  db	11001111b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]
; selector 0x0038
ldt: 
	  dw	0xFFFF		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10000010b	; P=1 DPL=00 DT=0 TYPE=0010
	  db	11001111b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]
; selector 0x0040
sys_tss: 
	  dw	103		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10001001b	; P=1 DPL=00 DT=0 TYPE=1001
	  db	11000000b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]
; selector 0x0048
user_tss1: 
	  dw	103		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10001001b	; P=1 DPL=00 DT=0 TYPE=1001
	  db	11000000b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]
; selector 0x0050
user_tss2: 
	  dw	103		; Limit 0..15 (4096*0FFFFFh=4GB)
	  dw	0		; Base 0..15
 	  db	0		; Base 16..23
	  db	10001001b	; P=1 DPL=00 DT=0 TYPE=1001
	  db	11000000b	; G=1 DB=1 res=0 av=0 Limit 16..19
	  db	0		; Base [24..31]

;SYSTEM_CODE_SELECTOR
;SYSTEM_TSS_SELECTOR
;USER_TSS_SELECTOR
; blah..blah
;ldt_selector:
;idt_selector: !!! this is really needed for DT=0 !!!
;tss_selector:

gdt_size  equ	$-gdt	
;=======================================================================

