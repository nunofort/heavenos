;------------------------------------------------------------
;Port-Index:  -               Port: Write/03c2h Read/03cch
;usage:       d7   Vertical sync polarity
;             d6   Horizontal sunc polarity
;             d5   Odd /even page
;             d4   Disable video
;             d3   Clock select 1
;             d2   Clock select 0
;             d1   Enable/Disable display RAM
;             d0   I/O address select
;Description: Sync polarity: Bits are set as below for VGA displays
;             that use sync polarity to determine screen resolution.
;             Many newer multiple frequency displays are insensitive
;             to sync polarity
;
;             d7 d6      Resolution
;             0  0       Invalid
;             0  1       400 lines
;             1  0       350 lines
;             1  1       480 lines
;
;             I/O address select: When set to zero, selects the
;             monochrome I/O address space (3bx). When set to one,
;             it selects the color I/O address space (3dx)
;
;------------------------------------------------------------
;Port-Index: -                Port: 03c2h ; read only
;usage:      d7    Vertical Retrace Interrupt pendling
;            d6    Feature connector bit 1
;            d5    Feature connector bit 0
;            d4    Switch sense
;            d0-d3 Unused
;
;Description: d7 uses IRQ2
;
;------------------------------------------------------------
;Port-Index: -                Port: 03bah,03dah ; read only
;usage:      d3  Vertical retrace
;            d0  Horizontal retrace
;
;------------------------------------------------------------
;Port-Index: -                Port: 03c3h,46e8h
;usage:      d7-d1  Reserved
;            d0     VGA enable/disable (03c3h only)
;
;Description: Disables access to display memmory and the other
;             VGA's ports
;
;------------------------------------------------------------
;Port-Index: 00h              Port: 03d4h, 03b4h
;usage:      Horizontal total
;Description: Total number of characters in horizontal scan minus
;             five ( including blanked and border characters)
;
;------------------------------------------------------------
;Port-Index: 01h              Port: 03d4h, 03b4h
;usage:      Horizontal display enable
;Description: Total number of characters displayed in horizontal
;             scan minus one.
;------------------------------------------------------------
;Port-Index: 02h              Port: 03d4h, 03b4h
;usage:      Start horizontal blanking
;Description: Character at which blanking starts
;
;------------------------------------------------------------
;Port-Index: 03h              Port: 03d4h, 03b4h
;usage:      End horizontal blanking
;            d7    Test
;            d6    Skew control
;            d5    Skew control
;            d0-d4 End blanking
;Description: End blanking: is five LSB bits of six-bit value,
;             which define the character at which blanking stops.
;             The MSB bit of this value is in register index 5.
;
;------------------------------------------------------------
;Port-Index: 04h              Port: 03d4h, 03b4h
;usage:      Start horizontal retrace
;Description: Character at which horizontal retrace starts
;
;------------------------------------------------------------
;Port-Index: 05h              Port: 03d4h, 03b4h
;usage:      End horizontal retrace
;            d7    End horizontal blanking bit 5
;            d6    Horizontal retrace delay
;            d5    Horizontal retrace delay
;            d0-d4 End horizontal retrace
;Description: End horizontal retrace: defines the character at
;             which horizontal retrace ends
;
;------------------------------------------------------------
;Port-Index: 06h              Port: 03d4h, 03b4h
;usage:      Vertical total
;Description: Total number of horizontal scan lines minus two
;             (including blanked and border characters). MSB bits
;             of this value are in register index 7
;
;------------------------------------------------------------
;Port-Index: 07h              Port: 03d4h, 03b4h
;usage:      Overflow register
;            d7  Vertical retrace start (bit 9)
;            d6  Vertical display enable end (bit 9)
;            d5  Vertical total (bit 9)
;            d4  Line compare (bit 8)
;            d3  Start vertical blank (bit 8)
;            d2  Vertical retrace start (bit 8)
;            d1  Vertical display enable end (bit 8)
;            d0  Vertical total (bit 8)
;------------------------------------------------------------
;Port-Index: 08h              Port: 03d4h, 03b4h
;usage:      Preset row scan
;            d7    Unused
;            d6    Byte panning control
;            d5    Byte panning control
;            d0-d4 Preset row scan
;Description: Byte panning control: is used to control byte
;             panning. This register together with attribute
;             controller register 13h allows for up to 31 pixels of
;             panning in double word modes
;             Preset row scan: Which character scan line is the
;             first to be displayed
;------------------------------------------------------------
;Port-Index: 09h              Port: 03d4h, 03b4h
;usage:      Maximum scan line/Character height
;            d7    double scan
;            d6    bit d9 of line compare register
;            d5    bit d9 of start vertical blank register
;            d0-d4 Maximum scan line
;Description: d0-d5=Character height-1,  only in textmodes
;------------------------------------------------------------
;Port-Index: 0ah              Port: 03d4h, 03b4h
;usage:      Cursor start
;            d7,d6 Reserved (0)
;            d5    Cursor off
;            d4-d0 Cursor start
;Description:
;------------------------------------------------------------
;Port-Index: 0bh              Port: 03d4h, 03b4h
;usage:      Cursor end
;            d7    reserved
;            d6,d5 Cursor skew
;            d4-d0 Cursor end
;Description:
;------------------------------------------------------------
;Port-Index: 0ch              Port: 03d4h, 03b4h
;usage:      Start address high
;------------------------------------------------------------
;Port-Index: 0dh              Port: 03d4h, 03b4h
;usage:      Start address low
;Description: Determine the offset in display memory to be
;             displayed on the upper-left corner on the screen
;------------------------------------------------------------
;Port-Index: 0eh              Port: 03d4h, 03b4h
;usage:      Cursor location (high byte)
;-----------------------CURSOR LOCATION----------------------
;Port-Index: 0fh              Port: 03d4h, 03b4h
;usage:      Cursor location (low byte)
;Description: Where the cursor is displayed on screen
;------------------------------------------------------------
;Port-Index: 10h              Port: 03d4h, 03b4h
;usage:      Vertical retrace start
;Description: 8 bits out of 10
;------------------------------------------------------------
;Port-Index: 11h              Port: 03d4h, 03b4h
;usage:      Vertical retrace end
;            d7    Write protect CRTC register 0 to 7
;            d6    refresh cycle select
;            d5    enable vertical interrupt (when 0)
;            d4    Clear vertical interrupt (when 0)
;            d0-d3 Vertical retrace end
;------------------------------------------------------------
;Port-Index: 12h              Port: 03d4h, 03b4h
;usage:      Vertical display enable end
;Description: eight LSB bits out of ten-bit value which define
;             scan line minus one at which the display ends.
;             The other two are in CRTC register index 7
;------------------------------------------------------------
;Port-Index: 13h              Port: 03d4h, 03b4h
;usage:      Offset / Logical screen width
;Description: Logical screen width between successive scan lines
;------------------------------------------------------------
;Port-Index: 14h              Port: 03d4h, 03b4h
;usage:      Underline location register
;            d7    Reserved
;            d6    Double word mode
;            d5    count by 4
;            d0-d4 Underline location
;Description: Underline location: Monochrome textmode only
;------------------------------------------------------------
;Port-Index: 15h              Port: 03d4h, 03b4h
;usage:      Start vertical blanking
;Description: eight LSB bits of ten-bit value minus one which
;             define at which scan line the vertical blanking
;             starts. The other two bits are in CRTC registers
;             index 7 and 9
;------------------------------------------------------------
;Port-Index: 16h              Port: 03d4h, 03b4h
;usage:      End vertical blanking
;Description: eight LSB bits of a value which determine the scan
;             line after which vertical blanking ends.
;------------------------------------------------------------
;Port-Index: 17h              Port: 03d4h, 03b4h
;usage:      Mode control register
;            d7  Enable vertical and hoizontal retrace
;            d6  Byte mode (1), word mode (0)
;            d5  Address wrap
;            d4  Reserved
;            d3  count by 2
;            d2  multiple vertical by 2 (use half in
;                CRTC (8,10,12,14,18)
;            d1  Select row scan counter (not used)
;            d0  compatibilty mode support (enable interleave)
;------------------------------------------------------------
;Port-Index: 18h              Port: 03d4h, 03b4h
;usage:      Line compare register
;Description: Split screen,  8 bit value out of a ten-bit value
;------------------------------------------------------------
;Port-Index: 00h              Port: 03c4h
;usage:      Reset register
;            d7-d2 Reserved
;            d1    Synchronous reset
;            d0    Asynchronous reset
;Description: Synchr. when set to zero, will halt and reset
;             the sequencer at the end of its current cycle
;             Asyncht. when set to zero, will immediatly halt
;             and reset the sequencer. Data can be loss.
;------------------------------------------------------------
;Port-Index: 01h              Port: 03c4h
;usage:      Clock mode register
;            d7,d6 Reserved
;            d5    display off
;            d4    Allow 32-bit Fetch (not used in standard modes)
;            d3    Divide dot clock by 2 (used in some 320*200 modes)
;            d2    Allow 16-bit fetch (used in mon graphics modes)
;            d1    Reserved
;            d0    Enable (0) 9 dot characters (mono text and 400-line)
;Description: Display off: Will blank screen and give the cpu
;             uninterrupted access the display memory.
;------------------------------------------------------------
;Port-Index: 02h              Port: 03c4h
;usage:      Color plane write enable register
;            d7,d6 Reserved
;            d3    Plane 3 Write enable
;            d2    Plane 2 Write enable
;            d1    Plane 1 Write enable
;            d0    Plane 0 Write enable
;Description:
;------------------------------------------------------------
;Port-Index: 03h              Port: 03c4h
;usage:      Character generator select register
;            d7,d6 Reserved
;            d5    Character generator table select A (MSB)
;            d4    Character generator table select B (MSB)
;            d3,d2 Character generator table select A
;            d1,d0 Character generator table select B
;Description: This register is only of interest if your software
;             will be using multiple character sets. Either one
;             or two character sets can be active. Table A selects
;             the charcater with attribute d3 set to zero and
;             Table B is the one with d3 set to one.
;------------------------------------------------------------
;Port-Index: 04h              Port: 03c4h
;usage:      Memory mode register
;            d4-d7 Reserved
;            d3    Chain 4 (address bits 0&1 to select plan, mode 13h)
;            d2    Odd/even (address bit 0 to select plane 0&2 or
;                  1&3 text modes)
;            d1    Extended memory (disable 64k modes)
;            d0    Reserved
;Description:
;------------------------------------------------------------
;Port-Index: 00h              Port: 03ceh
;usage:      Set / Reset register
;            d7-d4 Reserved (0)
;            d3    Fill data for plane 3
;            d2    Fill data for plane 2
;            d1    Fill data for plane 1
;            d0    Fill data for plane 0
;------------------------------------------------------------
;Port-Index: 01h              Port: 03ceh
;usage:      Set / Reset enable register
;            d7-d4 Reserved (0)
;            d3    enable set/reset for plane 3 (1 = enable)
;            d2    enable set/reset for plane 2 (1 = enable)
;            d1    enable set/reset for plane 1 (1 = enable)
;            d0    enable set/reset for plane 0 (1 = enable)
;Description: Set/Reset enable defines which memory planes will
;            receive fill data from set/reset register. Any plane
;             that is disable for set/reset will be written with
;             normal processor output data
;------------------------------------------------------------
;Port-Index: 02h              Port: 03ceh
;usage:      Color compare register
;            d7-d4 Reserved
;            d3    Color compare value for plane 3
;            d2    Color compare value for plane 2
;            d1    Color compare value for plane 1
;            d0    Color compare value for plane 0
;Description: one indicate that color is the same
;------------------------------------------------------------
;Port-Index: 03h              Port: 03ceh
;usage:      Data rotate / Function select register
;            d7-d5 Resrved (0)
;            d4,d3 Function select
;            d2-d0 Rotate count
;
;            d4 d3  Function
;            0  0   Write data unmodified
;            0  1   Write data ANDed with processor latches
;            1  0   Write data ORed with processor latches
;            1  1   Write data XORed with processor latches
;Description: Rotation is made before writing data
;------------------------------------------------------------
;Port-Index: 04h              Port: 03ceh
;usage:      Read plane select register
;            d7-d2 Reserved (0)
;            d1,d0 Defines color plane for reading (0-3)
;Description: Doesnt matter in color compare mode
;------------------------------------------------------------
;Port-Index: 05h              Port: 03ceh
;usage:      Mode register
;            d7    Reserved (0)
;            d6    256-colour mode
;            d5    Shift register mode
;            d4    Odd / Even mode
;            d3    Color compare mode enable (1 = enable)
;            d2    Reserved (0)
;            d1,d0 Write mode
;
;            d1 d0 Write mode
;            0  0  Direct write (data rotate, set/reset may apply)
;            0  1  Use processor latches as write data
;            1  0  Color plane n (0-3) is filled with the value of
;                  bit n in the write data
;            1  1  Use (rotated) write data ANDed with Bit mask as
;                  bit mask. Use set/reset as if set/reset was
;                  enable for all planes
;Description:
;------------------------------------------------------------
;Port-Index: 06h              Port: 03ceh
;usage:      Miscellaneous register
;            d7-d4 Reserved
;            d3-d2 Memory map
;                  00 = A000h for 128k
;                  01 = A000h for 64k
;                  10 = B000h for 32k
;                  11 = B800h for 32k
;            d1    Odd/even enable (used in text modes)
;            d0    Graphics mode enable
;Description: Memory map defines the location and size of the
;             host window
;------------------------------------------------------------
;Port-Index: 07h              Port: 03ceh
;usage:      Color don't care register
;            d7-d4 Reserved (0)
;            d3    Plane 3 don't care
;            d2    Plane 2 don't care
;            d1    Plane 1 don't care
;            d0    Plane 0 don't care
;Description: Color don't care is used in conjunction with color
;             compare mode. This register masks particular planes
;             from being tested during color compare cycles.
;------------------------------------------------------------
;Port-Index: 08h              Port: 03ceh
;usage:      Bitmask register
;Description: The bitmask register is used to mask certain bit
;             positons from being modified.
;------------------------------------------------------------
;Port-Index: -                 Port: 03c0h both index and data
;usage:      d7,d6 Reserved
;            d5    Palette address source
;                  0 = palette can be modified, screen is blanked
;                  1 = screen is enable, palette cannot be modified
;            d4-d0 Palette register address
;Description: Palette register address selects which register of
;             the attributes controller will be addres,sed by the
;             next I/O write cycle
;------------------------------------------------------------
;Port-Index: 00h-0fh          Port: 03c0h
;usage:      Color palette register
;            d6,d7 Reserved
;            d5-d0 Color value
;Description: not used in 256 color modes
;------------------------------------------------------------
;Port-Index: 10h              Port: 03c0h
;usage:      Mode control register
;            d7  p4,p5 source select
;            d6  pixel width
;            d5  Horizontal panning compatibility
;            d4  Reserved
;            d3  Background intensify / enable blinking
;            d2  Line graphics enable (text modes only)
;            d1  display type
;            d0  graphics / text mode
;Description: p4,p5 source select: selects the source for video
;              outputs p4 and p5 to the DACs. If set to zero, p4
;              and p5 are driven from the palette registers (normal
;              operation). If set to one, p4 and p5 video outputs
;              come from bits 0 and 1 of the color select register.
;             pixel width: is set to one in mode 13h (256-color mode)
;             horizontal panning compatibility: enhances the
;              operation of the line compare register of the CRT
;              controller, which allows one section of the screen
;              to be scrolled while another section remains stationary.
;              When this bit is set to one, the stationary
;              section of the screen will also be immune to horizontal
;              panning.
;------------------------------------------------------------
;Port-Index: 11h              Port: 03c0h
;usage:      Screen border color
;Description: In text modes, the screen border color register
;             selects the color of the border that sorrounds the
;             text display area on the screen. This is also referred
;             to by IBM as overscan. Unfortunately, this feature
;             does not work properly on EGA displays in 350-line
;             modes.
;------------------------------------------------------------
;Port-Index: 12h              Port: 03c0h
;usage:      Color plane enable register
;            d7,d6 Reserved
;            d5,d4 Video status mux
;            d3    Enable color plane 3
;            d2    Enable color plane 2
;            d1    Enable color plane 1
;            d0    Enable color plane 0
;Description:  The video status mux bits can be used in conjunction
;             with the diagnostic bits of input status register 1
;             to read palette registers. For the EGA, this is the
;             only means available for reading the palette registers.
;              Enable color planes can be used to enable or disable
;             color planes at the input to the color lockup table.
;             A zero in any of these bit positions will mask the
;             data from that color plane. The effect on the display
;             will be the same as if that color plane were cleared
;             to all zeros.
;------------------------------------------------------------
;Port-Index: 13h              Port: 03c0h
;usage:      Horizontal panning register
;            d7-d4 reserved
;            d3-d0 Horizontal pan
;Description: Horizontal pan allows the display to be shifted
;             horizontally one pixel at a time.
;
;             d3-d0      Number of pixels shifted to the left
;                        0+,1+,2+     13h     Other modes
;                        3+,7,7+
;             0          1            0       0
;             1          2            1       -
;             2          3            2       1
;             3          4            3       -
;             4          5            4       2
;             5          6            5       -
;             6          7            6       3
;             7          8            7       -
;             8          9            -       -
;------------------------------------------------------------
;Port-Index: 14h              Port: 03c0h
;usage:      Color select register
;            d7-d4 Reserved
;            d3    color 7
;            d2    color 6
;            d1    color 5
;            d0    color 4
;Description:  Color 7 and color 6: are normally used as the high
;             order bits of the eight-bit video color data from the
;             attribute controller to the DACs. The only exceptions
;             are 256-color modes
;              Color 5 and color 4: can be used in place of the p5
;             and p6 outputs from the palette registers (see mode
;             control register - index 10h). In 16-color modes, the
;             color select register can be used to rapidly cycle
;             between sets of colors in the video DAC.
;------------------------------------------------------------
;Port-Index: -                Port: 03c6h
;usage:      Pixel mask register
;Description: ???
;------------------------------------------------------------
;Port-Index: -                Port: 03c7h
;usage:      DAC state register (read-only)
;Description: if d0 and d1 is set to zero it indicates that
;             the lookup table is in a write mode
;------------------------------------------------------------
;Port-Index: -                Port: 03c7h
;usage:      Lookup table read index register (Write only)
;Description: Used when you want to read the palette (set color
;             number)
;------------------------------------------------------------
;Port-Index: -                Port: 03c8h
;usage:      Lookup table write index register
;Description: Used when you want to change palette (set color
;             number)
;------------------------------------------------------------
;Port-Index: -                Port: 03c9h
;usage:      Lookup table data register
;Description: Read color value (Red-Green-Blue) or write same data.
;------------------------------------------------------------


;%include "timer.inc"

VIDEO_TEXTMODE_clear_screen:
mov ecx, VIDEO_TEXTMODE_ADDRESS+4000
mov edi, VIDEO_TEXTMODE_ADDRESS
next_char:
mov dword [es:edi], 0
add edi, 4
cmp edi, ecx
jne next_char
;mov [VIDEO_TEXTMODE_ROW_POSITION], 0x01
retf

%macro VIDEO_TEXTMODE_typewriter	3
inc byte [VIDEO_TEXTMODE_LINE]
cmp byte [VIDEO_TEXTMODE_LINE], 24
jl %%skip_window_up
mov byte [VIDEO_TEXTMODE_WINDOW_LINE_START], 0x04
call dword 0x0008:VIDEO_TEXTMODE_window_up
dec byte [VIDEO_TEXTMODE_LINE]
%%skip_window_up:

mov edi, VIDEO_TEXTMODE_ADDRESS + 2*%1
mov eax, 160
xor ebx, ebx
mov bl, [VIDEO_TEXTMODE_LINE]
mul ebx
add edi, eax

mov esi, %3
mov ah, %2
%%next_char1:
;;; 
;;; ---  typewriter effect  ---
;;; TIMER_rtc_msdelay       50
;;; SPEAKER_sound
;;; 
mov al, byte [es:esi]
mov word [es:edi], ax
times 2 inc edi
inc esi
cmp byte [es:esi], 0x00
jne %%next_char1
%endmacro

%macro VIDEO_TEXTMODE_write_mensage	4
; LOOP (Loop While ECX Not Zero)
; CLD = clear direction flag (increment)
; LODSB = (mov al, byte [ds:esi] & inc esi)
; LEA EAX,[EBX+ECX*4+100] = real fast & complex arithmetic calculations
; MOVSB = (mov byte `[ES:EDI]' to byte `[DS:ESI]' & inc esi & inc edi)
; LES al, esi = (mov al, byte [es:esi])
; LES ax, edi = (mov ax, word [es:edi])
; LDS al, esi = (mov al, byte [ds:esi])
;
mov edi, VIDEO_TEXTMODE_ADDRESS + 2*%2

mov eax, 160
mov ebx, %1
mul ebx
add edi, eax

mov esi, %4
mov ah, %3
%%next_char:
mov al, byte [es:esi]
mov word [es:edi], ax
times 2 inc edi
inc esi
cmp byte [es:esi], 0x00
jne %%next_char
%endmacro

VIDEO_TEXTMODE_window_up:
mov esi, VIDEO_TEXTMODE_ADDRESS
mov edi, VIDEO_TEXTMODE_ADDRESS

mov eax, 160
xor ebx, ebx
mov bl, [VIDEO_TEXTMODE_WINDOW_LINE_START]
mul ebx
add edi, eax
add eax, 160
add esi, eax
mov ebx, VIDEO_TEXTMODE_ADDRESS+4000

next_copy:
mov eax, dword [es:esi]
mov dword [es:edi], eax
add esi, 4
add edi, 4
cmp esi, ebx
jne next_copy
next_clean:
mov dword [es:edi], 0x00000000
add edi, 4
cmp edi, ebx
jne next_clean 
retf

%macro VIDEO_TEXTMODE_set_line	1
mov al, %1
mov [VIDEO_TEXTMODE_LINE], al
%endmacro

%macro VIDEO_TEXTMODE_print_line	3

inc byte [VIDEO_TEXTMODE_LINE]
cmp byte [VIDEO_TEXTMODE_LINE], 24
jl %%skip_window_up
mov byte [VIDEO_TEXTMODE_WINDOW_LINE_START], 0x04 
call dword 0x0008:VIDEO_TEXTMODE_window_up
dec byte [VIDEO_TEXTMODE_LINE]
%%skip_window_up:

mov edi, VIDEO_TEXTMODE_ADDRESS + 2*%1
mov eax, 160
xor ebx, ebx
mov bl, [VIDEO_TEXTMODE_LINE]
mul ebx
add edi, eax

mov esi, %3
mov ah, %2
%%next_char1:
mov al, byte [es:esi]
mov word [es:edi], ax
times 2 inc edi
inc esi
cmp byte [es:esi], 0x00
jne %%next_char1
%endmacro

; COLOR, R, G, B
%macro VIDEO_VGA_get_pal	4
cli
pushf
mov dx, VIDEO_VGA_READ_INDEX_PORT
mov al, [%1]
out dx, al
mov dx, VIDEO_VGA_DATA_PORT
in al, dx
mov byte [%2], al
in al, dx
mov byte [%3], al
in al, dx
mov byte [%4], al
popf
sti
%endmacro 

; COLOR, R, G, B
%macro VIDEO_VGA_set_pal	4
cli
pushf
mov dx, VIDEO_VGA_WRITE_INDEX_PORT
mov al, %1
out dx, al
mov dx, VIDEO_VGA_DATA_PORT
mov al, %2
out dx, al 
mov al, %3
out dx, al
mov al, %4
out dx, al
popf
sti
%endmacro

%macro VIDEO_VGA_wait_retrace	0
cli
pushf
mov dx, 0x03DA
%%loop1:
in al, dx
and al, 0x08
jnz near %%loop1
%%loop2:
in al, dx
and al, 0x08
jz near %%loop2
popf
sti
%endmacro

%macro VIDEO_VGA_save_pal	0
mov edi, VIDEO_VGA_PAL_BUFFER
mov byte [VIDEO_VGA_COLOR], 0
%%get_next_color:
VIDEO_VGA_get_pal      VIDEO_VGA_COLOR ,VIDEO_VGA_RED,VIDEO_VGA_GREEN,VIDEO_VGA_BLUE
mov al, [VIDEO_VGA_RED]
mov byte [es:edi], al
mov al, [VIDEO_VGA_GREEN]
mov byte [es:edi+1], al
mov al, [VIDEO_VGA_BLUE]
mov byte [es:edi+2], al
add edi, 3
inc byte [VIDEO_VGA_COLOR]
cmp byte [VIDEO_VGA_COLOR], 16
jne %%get_next_color
%endmacro

VIDEO_VGA_fadeout:
VIDEO_VGA_save_pal
set_next_dec:
mov byte [VIDEO_VGA_COLOR], 0
mov byte [VIDEO_VGA_RGB], 0
VIDEO_VGA_wait_retrace
set_next_color:
VIDEO_VGA_get_pal        VIDEO_VGA_COLOR ,VIDEO_VGA_RED,VIDEO_VGA_GREEN,VIDEO_VGA_BLUE
cmp byte [VIDEO_VGA_RED], 0
je skip_red
inc byte [VIDEO_VGA_RGB]
dec byte [VIDEO_VGA_RED]
skip_red:
cmp byte [VIDEO_VGA_GREEN], 0
je skip_green
inc byte [VIDEO_VGA_RGB]
dec byte [VIDEO_VGA_GREEN]
skip_green:
cmp byte [VIDEO_VGA_BLUE], 0
je skip_blue
inc byte [VIDEO_VGA_RGB]
dec byte [VIDEO_VGA_BLUE]
skip_blue:
VIDEO_VGA_set_pal	[VIDEO_VGA_COLOR],[VIDEO_VGA_RED],[VIDEO_VGA_GREEN],[VIDEO_VGA_BLUE]
inc byte [VIDEO_VGA_COLOR]
cmp byte [VIDEO_VGA_COLOR], 16 
je end_dec
jmp dword 0x0008:set_next_color
end_dec:
TIMER_rtc_msdelay       50 
cmp byte [VIDEO_VGA_RGB], 0
je end_fadeout
jmp dword 0x0008:set_next_dec
end_fadeout:
retf

VIDEO_VGA_fadein:
set_next_inc:
mov esi, VIDEO_VGA_PAL_BUFFER
mov byte [VIDEO_VGA_COLOR], 0
mov byte [VIDEO_VGA_RGB], 0
VIDEO_VGA_wait_retrace
set_next_color1:
VIDEO_VGA_get_pal        VIDEO_VGA_COLOR ,VIDEO_VGA_RED,VIDEO_VGA_GREEN,VIDEO_VGA_BLUE
mov al, byte [es:esi]
cmp byte [VIDEO_VGA_RED], al 
je skip_red1
inc byte [VIDEO_VGA_RGB]
inc byte [VIDEO_VGA_RED]
skip_red1:
mov al, byte [es:esi+1]
cmp byte [VIDEO_VGA_GREEN], al 
je skip_green1
inc byte [VIDEO_VGA_RGB]
inc byte [VIDEO_VGA_GREEN]
skip_green1:
mov al, byte [es:esi+2]
cmp byte [VIDEO_VGA_BLUE], al 
je skip_blue1
inc byte [VIDEO_VGA_RGB]
inc byte [VIDEO_VGA_BLUE]
skip_blue1:
VIDEO_VGA_set_pal	[VIDEO_VGA_COLOR],[VIDEO_VGA_RED],[VIDEO_VGA_GREEN],[VIDEO_VGA_BLUE]
add esi,3
inc byte [VIDEO_VGA_COLOR]
cmp byte [VIDEO_VGA_COLOR], 16 
je end_inc
jmp dword 0x0008:set_next_color1
end_inc:
TIMER_rtc_msdelay       50 
cmp byte [VIDEO_VGA_RGB], 0
je end_fadein
jmp dword 0x0008:set_next_inc
end_fadein:
retf

;============================================================================
VIDEO_TEXTMODE_ADDRESS		equ     0x000B8000
VIDEO_GRAPHMODE_ADDRESS		equ	0x000A0000
VIDEO_TEXTMODE_ROW_POSITION	DB	0x01
VIDEO_TEXTMODE_COL_POSITION	DB	0x01
VIDEO_VGA_READ_INDEX_PORT	equ	0x03C7
VIDEO_VGA_WRITE_INDEX_PORT	equ	0x03C8
VIDEO_VGA_DATA_PORT		equ	0x03C9
VIDEO_VGA_PALETTE_SIZE		equ	256 * 3
VIDEO_VGA_PAL_BUFFER		times VIDEO_VGA_PALETTE_SIZE DB 0
VIDEO_VGA_RED			DB	0
VIDEO_VGA_GREEN			DB	0
VIDEO_VGA_BLUE			DB	0
VIDEO_VGA_COLOR			DB	0
VIDEO_VGA_RGB			DB	0
VIDEO_TEXTMODE_LINE		DB	0
VIDEO_TEXTMODE_WINDOW_LINE_START	DB	0	

;offset = (row * ROW_RES ) + column
;320x200 - 64K memory

; !!! console lib !!!
; before writing a line to the bottom screen, screen must scroll one line up
; if is something on it

