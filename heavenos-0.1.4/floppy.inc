;NEC æPD765  -  Floppy Disk Controller  -  8272A
;
;       PS/2 FDC Diskette Status Register A at 3F0h
;
;        ³7³6³5³4³3³2³1³0³  3F0h  PS/2 Disk Status Register A (read-only)
;         ³ ³ ³ ³ ³ ³ ³ ÀÄÄÄÄ direction
;         ³ ³ ³ ³ ³ ³ ÀÄÄÄÄÄ write protect
;         ³ ³ ³ ³ ³ ÀÄÄÄÄÄÄ index
;         ³ ³ ³ ³ ÀÄÄÄÄÄÄÄ head 1 select
;         ³ ³ ³ ÀÄÄÄÄÄÄÄÄ track 0
;         ³ ³ ÀÄÄÄÄÄÄÄÄÄ step
;         ³ ÀÄÄÄÄÄÄÄÄÄÄ second drive installed
;         ÀÄÄÄÄÄÄÄÄÄÄÄ interrupt pending
;
;       PS/2 FDC Diskette Status Register B at 3F1h
;
;        ³7³6³5³4³3³2³1³0³  3F1h  PS/2 Disk Status Register B (read-only)
;         ³ ³ ³ ³ ³ ³ ³ ÀÄÄÄÄ motor enable 0
;         ³ ³ ³ ³ ³ ³ ÀÄÄÄÄ motor enable 1
;         ³ ³ ³ ³ ³ ÀÄÄÄÄ write enable
;         ³ ³ ³ ³ ÀÄÄÄÄ read data (toggles w/positive transition in -RD DATA)
;         ³ ³ ³ ÀÄÄÄÄ write data (toggles w/positive transition in WR DATA)
;         ³ ³ ÀÄÄÄÄ drive select
;         ÀÄÁÄÄÄÄ reserved
;
;       FDC Digital Output Register at 3F2h  (all systems)
;
;        ³7³6³5³4³3³2³1³0³  port 3F2h  (write only)
;         ³ ³ ³ ³ ³ ³ ÀÄÁÄÄÄÄ floppy drive select (0=A, 1=B, 2=floppy C, ...)
;         ³ ³ ³ ³ ³ ÀÄÄÄÄÄÄÄ 1 = FDC enable, 0 = hold FDC at reset
;         ³ ³ ³ ³ ÀÄÄÄÄÄÄÄÄ 1 = DMA & I/O interface enabled  (reserved PS/2)
;         ³ ³ ³ ÀÄÄÄÄÄÄÄÄÄ 1 = turn floppy drive A motor on
;         ³ ³ ÀÄÄÄÄÄÄÄÄÄÄ 1 = turn floppy drive B motor on
;         ³ ÀÄÄÄÄÄÄÄÄÄÄÄ 1 = turn floppy drive C motor on; (reserved PS/2)
;         ÀÄÄÄÄÄÄÄÄÄÄÄÄ 1 = turn floppy drive D motor on; (reserved PS/2)
;
;        - used to control drive motors, drive selection, and feature enable
;        - PS/2 only uses bit 0 for floppy drive select; bit 1 is reserved
;        - PS/2 only uses bits 5 & 4 for  motor enable; bits 7&6 are reserved
;        - all DOR bits are cleared during controller reset
;
;       FDC Main Status Register at 3F4h  (all systems)
;
;        ³7³6³5³4³3³2³1³0³  port 3F4h  (read only)
;         ³ ³ ³ ³ ³ ³ ³ ÀÄÄÄÄ floppy drive 0 in seek mode/busy
;         ³ ³ ³ ³ ³ ³ ÀÄÄÄÄÄ floppy drive 1 in seek mode/busy
;         ³ ³ ³ ³ ³ ÀÄÄÄÄÄÄ floppy drive 2 in seek mode/busy (reserved PS/2)
;         ³ ³ ³ ³ ÀÄÄÄÄÄÄÄ floppy drive 3 in seek mode/busy (reserved PS/2)
;         ³ ³ ³ ÀÄÄÄÄÄÄÄÄ FDC read or write command in progress
;         ³ ³ ÀÄÄÄÄÄÄÄÄÄ FDC is in non-DMA mode
;         ³ ÀÄÄÄÄÄÄÄÄÄÄ I/O direction;  1 = FDC to CPU; 0 = CPU to FDC
;         ÀÄÄÄÄÄÄÄÄÄÄÄ data reg ready for I/O to/from CPU (request for master)
;
;       FDC Command Status Register 0 at 3F5h  (all systems)
;
;        ³7³6³5³4³3³2³1³0³  Command Status Register 0 at port 3F5h
;         ³ ³ ³ ³ ³ ³ ÀÄÁÄÄÄÄ unit selected at interrupt (0=A, 1=B, 2=...)
;         ³ ³ ³ ³ ³ ÀÄÄÄÄÄÄÄ head number at interrupt (head 0 or 1)
;         ³ ³ ³ ³ ÀÄÄÄÄÄÄÄÄ not ready on read/write or SS access to head 1
;         ³ ³ ³ ÀÄÄÄÄÄÄÄÄÄ equipment check (see note)
;         ³ ³ ÀÄÄÄÄÄÄÄÄÄÄ set to 1 when FDD completes a seek command
;         ÀÄÁÄÄÄÄÄÄÄÄÄÄÄ last command status (see below)
;
;        Bits
;         76  Last Command Status
;         00  command terminated successfully
;         01  command execution started but terminated abnormally
;         10  invalid command issued
;         11  command terminated abnormally due to a change in state of
;             the Ready Signal from the FDC  (reserved on PS/2)
;
;        - equipment check can occur if FDD signals a fault or track zero is
;          not found after 77 steps on a recalibrate command
;        - PS/2 only uses bits 1-0 for drive (values are 01b and 10b)
;
;        FDC Command Status Register 1 at 3F5h  (all systems)
;
;        ³7³6³5³4³3³2³1³0³  Command Status Register 1 at port 3F5h
;         ³ ³ ³ ³ ³ ³ ³ ÀÄÄÄÄ FDC cannot find ID address mark (see reg 2)
;         ³ ³ ³ ³ ³ ³ ÀÄÄÄÄÄ write protect detected during write
;         ³ ³ ³ ³ ³ ÀÄÄÄÄÄÄ FDC cannot find sector ID
;         ³ ³ ³ ³ ÀÄÄÄÄÄÄÄ unused (always zero)
;         ³ ³ ³ ÀÄÄÄÄÄÄÄÄ over-run;  FDC not serviced in reasonable time
;         ³ ³ ÀÄÄÄÄÄÄÄÄÄ data error (CRC) in ID field or data field
;         ³ ÀÄÄÄÄÄÄÄÄÄÄ unused (always zero)
;         ÀÄÄÄÄÄÄÄÄÄÄÄ end of cylinder; sector# greater than sectors/track
;
;        - bit 0 of Status Register 1 and bit 4 of Status Register 2 are
;          related and mimic each other
;
;        FDC Command Status Register 2 at 3F5h  (all systems)
;
;        ³7³6³5³4³3³2³1³0³  Command Status Register 2 at port 3F5h
;         ³ ³ ³ ³ ³ ³ ³ ÀÄÄÄÄ missing address mark in data field
;         ³ ³ ³ ³ ³ ³ ÀÄÄÄÄÄ bad cylinder, ID not found and Cyl Id=FFh
;         ³ ³ ³ ³ ³ ÀÄÄÄÄÄÄ scan command failed, sector not found in cylinder
;         ³ ³ ³ ³ ÀÄÄÄÄÄÄÄ scan command equal condition satisfied
;         ³ ³ ³ ÀÄÄÄÄÄÄÄÄ wrong cylinder detected
;         ³ ³ ÀÄÄÄÄÄÄÄÄÄ CRC error detected in sector data
;         ³ ÀÄÄÄÄÄÄÄÄÄÄ sector with deleted data address mark detected
;         ÀÄÄÄÄÄÄÄÄÄÄÄ unused (always zero)
;
;        - bit 0 of Status Register 1 and bit 4 of Status Register 2 are
;          related and mimic each other
;
;        FDC Command Status Register 3 at 3F5h (FDD status, all systems)
;
;        ³7³6³5³4³3³2³1³0³  Floppy Disk Drive Status at port 3F5h
;         ³ ³ ³ ³ ³ ³ ÀÄÁÄÄÄÄ FDD unit selected status (0=A, 1=B, 2=...)
;         ³ ³ ³ ³ ³ ÀÄÄÄÄÄÄÄ FDD side head select status (0=head 0, 1=head 1)
;         ³ ³ ³ ³ ÀÄÄÄÄÄÄÄÄ FDD two sided status signal
;         ³ ³ ³ ÀÄÄÄÄÄÄÄÄÄ FDD track zero status signal
;         ³ ³ ÀÄÄÄÄÄÄÄÄÄÄ FDD ready status signal
;         ³ ÀÄÄÄÄÄÄÄÄÄÄÄ FDD write protect status signal
;         ÀÄÄÄÄÄÄÄÄÄÄÄÄ FDD fault status signal
;
;        PS/2 FDC Digital Input Register at 3F7h
;
;        ³7³6³5³4³3³2³1³0³  3F7h  PS/2 Digital Input Register (read only)
;         ³ ³ ³ ³ ³ ³ ³ ÀÄÄÄ high density select
;         ³ ÀÄÁÄÁÄÁÄÁÄÁÄÄÄÄ reserved
;         ÀÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ diskette change
;
;        PS/2 FDC Configuration Control Register at 3F7h
;
;        ³7³6³5³4³3³2³1³0³  3F7h  PS/2 Config. Control Register (write only)
;         ³ ³ ³ ³ ³ ³ ÀÄÁÄÄ DRC1, DRC0  (see below)
;         ÀÄÁÄÁÄÁÄÁÄÁÄÄÄÄÄ reserved
;
;        DRC1 DRC0
;          0   0    500000 bit per second mode
;          0   1    reserved
;          1   0    250000 bit per second mode
;          1   1    reserved
;
;        - Digital Input Register is used to sense the state of the
;          (-diskette change) and the (-high density select) signals
;        - Configuration Control Register is used to set the transfer rate
;
;
;FDC Programming Considerations
;
;        Three phases of command execution:
;
;        1.  Command phase; commands are sent from the CPU to the FDC via
;            port 3F5h;  bit 6 of the Status Register at 3F4h must be zero
;        2.  Execution phase; FDC executes instruction & generates INT 6
;        3.  Result phase; status and other information is available to CPU;
;            INT 6 sets bit 7 of BIOS Data Area location 40:3E which can
;            be polled for completion status
;
;
;        Example of a read operation:
;
;        1. turn disk motor on and set delay time for drive spin up
;        2. perform seek operation;  wait for disk interrupt
;        3. prepare DMA chip to move data to memory
;        4. send read command and wait for transfer complete interrupt
;        5. read status information
;        6. turn disk motor off
;
;Floppy Diskette Controller Operations (15 commands)
;
;        Read Data          D7  D6  D5  D4  D3  D2  D1  D0
;
;        command byte 0:    MT  MF  SK   0   0   1   1   0
;        command byte 1:     ?   ?   ?   ?   ?  HD  US1 US0
;        command byte 2:    cylinder number
;        command byte 3:    head number
;        command byte 4:    sector number
;        command byte 5:    bytes per sector
;        command byte 6:    end of track (last sector in track)
;        command byte 7:    gap 3 length
;        command byte 8:    data length (if cmd byte 5==0)
;        result byte 0:     status register 0
;        result byte 1:     status register 1
;        result byte 2:     status register 2
;        result byte 3:     cylinder number
;        result byte 4:     head number
;        result byte 5:     sector number
;        result byte 6:     bytes per sector
;
;        Write Data         D7  D6  D5  D4  D3  D2  D1  D0
;
;        command byte 0:    MT  MF   0   0   0   1   0   1
;        command byte 1:     ?   ?   ?   ?   ?  HD  US1 US0
;        command byte 2:    cylinder number
;        command byte 3:    head number
;        command byte 4:    sector number
;        command byte 5:    bytes per sector
;        command byte 6:    end of track (last sector in track)
;        command byte 7:    gap 3 length
;        command byte 8:    data length (if cmd byte 5==0)
;        result byte 0:     status register 0
;        result byte 1:     status register 1
;        result byte 2:     status register 2
;        result byte 3:     cylinder number
;        result byte 4:     head number
;        result byte 5:     sector number
;        result byte 6:     bytes per sector
;
;        Read a Track       D7  D6  D5  D4  D3  D2  D1  D0
;        (Diagnostic)
;        command byte 0:     0  MF  SK   0   0   0   1   0
;        command byte 1:     ?   ?   ?   ?   ?  HD  US1 US0
;        command byte 2:    cylinder number
;        command byte 3:    head number
;        command byte 4:    sector number
;        command byte 5:    bytes per sector
;        command byte 6:    end of track (last sector in track)
;        command byte 7:    gap 3 length
;        command byte 8:    data length (if cmd byte 5==0)
;        result byte 0:     status register 0
;        result byte 1:     status register 1
;        result byte 2:     status register 2
;        result byte 3:     cylinder number
;        result byte 4:     head number
;        result byte 5:     sector number
;        result byte 6:     bytes per sector
;
;        Format a Track     D7  D6  D5  D4  D3  D2  D1  D0
;        (Write Sector IDs)
;        command byte 0:     0  MF   0   0   1   1   0   1
;        command byte 1:     ?   ?   ?   ?   ?  HD  US1 US0
;        command byte 2:    bytes per sector
;        command byte 3:    sectors per track
;        command byte 4:    gap 3 length
;        command byte 5:    filler pattern to write in each byte
;        result byte 0:     status register 0
;        result byte 1:     status register 1
;        result byte 2:     status register 2
;        result byte 3:     cylinder number
;        result byte 4:     head number
;        result byte 5:     sector number
;        result byte 6:     bytes per sector
;
;        Recalibrate        D7  D6  D5  D4  D3  D2  D1  D0
;
;        command byte 0:     0   0   0   0   0   1   1   1
;        command byte 1:     ?   ?   ?   ?   ?   0  US1 US0
;	 returns nothing
;
;        Seek               D7  D6  D5  D4  D3  D2  D1  D0
;
;        command byte 0:     0   0   0   0   1   1   1   1
;        command byte 1:     ?   ?   ?   ?   ?  HD  US1 US0
;        command byte 2:    new cylinder number
;        returns nothing
;

;%include "dma.inc"
;%include "tss.inc"

%macro FLOPPY_read_main_status		0
mov dx,FLOPPY_MAIN_STATUS_REGISTER
in al,dx
; status in AL
%endmacro

%macro FLOPPY_digital_output_write	1
mov dx, FLOPPY_DIGITAL_OUTPUT_REGISTER
mov al, %1
out dx, al
%endmacro

%macro FLOPPY_digital_input_read	0
mov dx, FLOPPY_DIGITAL_INPUT_REGISTER
in al,dx
; status in AL
%endmacro

%macro FLOPPY_config_control_write	1
mov dx, FLOPPY_CONFIG_CONTROL_REGISTER
mov al, %1
out dx, al
%endmacro

%macro FLOPPY_wait_fdc	0	
%%wait_fdc:
FLOPPY_read_main_status
test al, 0x80	; 8th bit is set ?
jz %%wait_fdc
%endmacro

%macro FLOPPY_command_write	1
FLOPPY_wait_fdc
mov dx, FLOPPY_COMMAND_STATUS_REGISTER
mov al, %1
out dx, al
%endmacro

%macro FLOPPY_command_read	1
FLOPPY_wait_fdc
mov dx, FLOPPY_COMMAND_STATUS_REGISTER
in al, dx
mov byte [%1], al
%endmacro

%macro FLOPPY_turn_on	0
FLOPPY_digital_output_write	00011100b	; enable motor A, DMA, FDC, Drive A
;FLOPPY_digital_output_write	00010100b	; enable motor A, Non-DMA, FDC, Drive A
TIMER_rtc_msdelay		1000		; Waiting while motor speeds up
%endmacro

%macro FLOPPY_turn_off	0
FLOPPY_digital_output_write	00001100b	; disable motor A, DMA, FDC, Drive A
;FLOPPY_digital_output_write	00000100b	; disable motor A, Non-DMA, FDC, Drive A
TIMER_rtc_msdelay		1000		; Waiting while motor speeds up
%endmacro

%macro FLOPPY_calibrate	0
FLOPPY_command_write    00000111b
FLOPPY_command_write    00000000b		; (00) Drive A
FLOPPY_wait_int					; wait for interrupt
; no result
%endmacro

%macro FLOPPY_seek		2
FLOPPY_command_write    00001111b	; Seek
;FLOPPY_command_write    10001111b	; Seek Relative
FLOPPY_command_write    %1       	; Drive/Head
FLOPPY_command_write    %2       	; Cyl
FLOPPY_wait_int				; wait for interrupt
;TIMER_rtc_msdelay		300	; delay for head positioning
; no result
%endmacro

%macro FLOPPY_read_track	3
FLOPPY_seek             %1, %2
DMA_setup       2,  0x90000+(512*18*2*%2)+(512*18*%3), 512*18
;;; COMMAND PHASE
FLOPPY_command_write    01100010b       ; M(2heads), FM, SK, READ TRACK
FLOPPY_command_write    %1		; head, drive
FLOPPY_command_write    %2            	; cylinder(track)
FLOPPY_command_write    %3            	; head 
FLOPPY_command_write    0x01            ; sector 1, not used (ignored)
FLOPPY_command_write	0x02		; bytes/sector = 512
FLOPPY_command_write	0x12		; sectors/track = 18
FLOPPY_command_write	0x1B		; GAP 3 for 1.44 = 27
FLOPPY_command_write	0xFF		; sector size is not zero
FLOPPY_wait_int				; Waiting for interrupt
;;; RESULT PHASE (We really need this stuff!)
FLOPPY_command_read	FLOPPY_RESULT0
FLOPPY_command_read	FLOPPY_RESULT1
FLOPPY_command_read	FLOPPY_RESULT2
FLOPPY_command_read	FLOPPY_RESULT3
FLOPPY_command_read	FLOPPY_RESULT4
FLOPPY_command_read	FLOPPY_RESULT5
FLOPPY_command_read	FLOPPY_RESULT6

;TIMER_rtc_msdelay		1000	; delay for head positioning
%endmacro

%macro FLOPPY_read_data		4	
FLOPPY_seek             %1, %2
;;; Setup DMA before this..512*18 each time
FLOPPY_command_write 	01100110b	; FM, SK, READ DATA 
FLOPPY_command_write	%1		; head, drive
FLOPPY_command_write	%2		; cylinder(track)
FLOPPY_command_write	%3		; head
FLOPPY_command_write	%4		; sector
FLOPPY_command_write	0x02		; bytes/sector = 512
FLOPPY_command_write	0x12		; sectors/track = 18
FLOPPY_command_write	0x1B		; GAP 3 for 1.44 = 27
FLOPPY_command_write	0xFF		; sector size is not zero
FLOPPY_wait_int				; Waiting for interrupt
FLOPPY_command_read	FLOPPY_RESULT0
FLOPPY_command_read	FLOPPY_RESULT1
FLOPPY_command_read	FLOPPY_RESULT2
FLOPPY_command_read	FLOPPY_RESULT3
FLOPPY_command_read	FLOPPY_RESULT4
FLOPPY_command_read	FLOPPY_RESULT5
FLOPPY_command_read	FLOPPY_RESULT6
%endmacro

%macro FLOPPY_interrupt_status	0
FLOPPY_command_write 	00001000b	; check interrupt status 
FLOPPY_command_read	FLOPPY_RESULT0
FLOPPY_command_read	FLOPPY_RESULT1
%endmacro

%macro FLOPPY_wait_int	0
%%wait_int:				; we are waiting for interrupt event 
cmp byte [FLOPPY_INTERRUPT], 1
jne %%wait_int
mov byte [FLOPPY_INTERRUPT], 0
%endmacro

;%1 START_TRACK
;%2 START_HEAD
;%3 START_ADDRESS
;%4 LENGTH
%macro FLOPPY_copy_to_address	 0	
FLOPPY_turn_on                          ; Turn on A floppy drive motor
FLOPPY_config_control_write	0	; Setting Data Transfer to 500 KBits/sec
FLOPPY_calibrate			; Calibrate Command
;;;DMA_setup       2,  0x90000, 512*18*6
FLOPPY_read_track       0, 0, 0     ; Send "Read Track" Command
FLOPPY_read_track       4, 0, 1
FLOPPY_read_track       0, 1, 0
FLOPPY_read_track       4, 1, 1
FLOPPY_read_track       0, 2, 0
FLOPPY_read_track       4, 2, 1
FLOPPY_turn_off                          ; Turn off A floppy drive motor
%endmacro

; TODO: after a "read/write command" we should check 
;       if the operation was successfull !
;       FLOPPY_interrupt_status ST0 bit 7 and 6 = 00 when success
;============================================================================;
; FDC 8272A (Floppy Disk Controller)                                         ;
;============================================================================;
FLOPPY_INTERRUPT		DB	0
; "read interrupt state", "read media/drive state" commands
FLOPPY_RESULT0			DB	0
FLOPPY_RESULT1			DB	0
FLOPPY_RESULT2			DB	0
FLOPPY_RESULT3			DB	0
FLOPPY_RESULT4			DB	0
FLOPPY_RESULT5			DB	0
FLOPPY_RESULT6			DB	0
FLOPPY_MAIN_STATE		DB	0
FLOPPY_MAIN_STATUS_REGISTER	equ	0x3F4
FLOPPY_COMMAND_STATUS_REGISTER	equ	0x3F5
FLOPPY_DIGITAL_OUTPUT_REGISTER	equ	0x3F2
FLOPPY_DIGITAL_INPUT_REGISTER	equ	0x3F7
FLOPPY_CONFIG_CONTROL_REGISTER	equ	0x3F7
FLOPPY_TEMP_VALUE		DB	0
FLOPPY_DPT_ADDRESS		equ 0x78	; BIOS Offset
FLOPPY_DRIVE_PARAMETER_TABLE:
				DB	0	; srt_hut
				DB	0	; dma_hlt
				DB	0	; motor_w
				DB	0	; sec_size
				DB	0	; eot
				DB	0	; gap_rw
				DB	0	; dtl
				DB	0	; gap_f
				DB	0	; fill_char
				DB	0	; hst
				DB	0	; mot_start
				DB	0	; not used, only for help
;===========================================================================

