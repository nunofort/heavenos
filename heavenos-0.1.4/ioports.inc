; Hardware input/output port access functions

%macro in_byte	2
mov dx, %1 
mov al, %2
in al, dx
mov %2, al
%endmacro

%macro out_byte	2
mov dx, %1 
mov al, %2
out dx, al 	
%endmacro
