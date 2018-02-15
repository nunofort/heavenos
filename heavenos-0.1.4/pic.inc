
%macro PIC_init1	1
mov al, %1				; bit0 = 1, init4 is espected?
					; bit1 = 0, cascaded PIC's?
					; bit2..3 = 0
					; bit4 = 1
					; bit5..7 = 0
out PIC_MASTER_PORT0,al
out PIC_SLAVE_PORT0,al
%endmacro

; set base vector number for IRQ's
%macro PIC_init2	2
mov al, %1				; bit0..2 = 0
					; bit3..7 = address of IRQ0 in IDT
					;           desc (multiple of 8h)
					; for IRQn = INT (desc+n) is called 
out PIC_MASTER_PORT1,al			; For master PIC
mov al, %2				; bit3..7 = address of IRQ8 in IDT
out PIC_SLAVE_PORT1,al			; For slave PIC
%endmacro

; if bit 1 of init1 was cleared,
; establish the connection between the PIC's
%macro PIC_init3	2
mov al, %1				; bitn = 0, IRQn connected to peripheral
					; bitn = 1, IRQn connected to Slave PIC
					; IRQ2 usually connected to Slave PIC
					
out PIC_MASTER_PORT1,al
mov al, %2				; bit0..2 = IRQ Number of the master
					; to which the slave is connected
					; usually IRQ Number 2
					; bit3..7 = 0
out PIC_SLAVE_PORT1,al
%endmacro

; if bit 0 of init1 was set
%macro PIC_init4	1
mov al, %1				; bit0 = 1, working on 80x86 CPU?
					; bit1 = 0, manual EOI
					; End of Interruption by Software?
					; bit2..7 = 0
out PIC_MASTER_PORT1,al
out PIC_SLAVE_PORT1,al
%endmacro

%macro PIC_master_set_irqs	1
mov al, %1				; bitn = 0, IRQn enabled?
					; with IRQ2 also Slave PIC is set!
out PIC_MASTER_PORT1,al			
%endmacro

%macro PIC_slave_set_irqs	1
mov al, %1				; bitn = 0, IRQn enabled?
out PIC_SLAVE_PORT1,al
%endmacro

%macro PIC_master_get_irqs	1
in al, PIC_MASTER_PORT1
mov %1, al				; bitn = 0, IRQn enabled?
					; with IRQ2 also Slave PIC is set!
%endmacro

%macro PIC_slave_get_irqs	1
in al, PIC_SLAVE_PORT1
mov %1, al				; bitn = 0, IRQn enabled?
%endmacro

; for manual EOI from the Interrupt Routine
; send a "End of Interrupt" command to the PIC
%macro PIC_eoi_cmd		0
mov al, 0x20				; EOI Command
out PIC_SLAVE_PORT0,al			; when IRQ comes from slave PIC
out PIC_MASTER_PORT0,al			; when IRQ comes from master PIC
; the two cases have to be done, when the IRQ comes from slave PIC,
; since the IRQ of the slave are sent through IRQ2 of the master
%endmacro

PIC_setup:
PIC_init1	00010001b
; IRQ0 = IDT Entry 32
PIC_init2	00100000b, 00101000b
PIC_init3	00000100b, 00000010b
PIC_init4	00000001b
; Disable all IRQ's except Timer IRQ0, Keyboard IRQ1, Floppy IRQ6 and RTC IRQ8
PIC_master_set_irqs	10111000b
PIC_slave_set_irqs	11111110b
retf

;=======================================================================;
; PIC-8259 (Programmable Interrupt Controller)				;
;=======================================================================;
PIC_MASTER_PORT0	equ	0x20
PIC_MASTER_PORT1	equ	0x21
PIC_SLAVE_PORT0		equ	0xA0
PIC_SLAVE_PORT1		equ	0xA1


