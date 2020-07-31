
![ScreenShot](heavenos_shot.jpg)


=============[ ABOUT THE PROJECT

At some years ago, i was trying to develop an operating system with help 
of some buddy hackers (not evil guys..as you may think of that word, 
but just normal people who like to hack code, study algorithms and technologies, 
of computing environments as myself) on the internet.
Unfortunately, the development was locked because of an annoying 
hardware floppy disk issue and in addition, i became to have a bit 
less time to work on it. 
At that stage, the development was stopped!
So, in hope someone want to see and improve the source code on what is done in the 
operating system, the source code is now released to the public.
Feel free to download and experiment the code. At the bottom of this document is 
information how to get it to work.
Well..if you have some knowledge on the subject, 
i will be glad to see future enhancements on the OS code. 



============[ BACKGROUND MESSAGE (OUTDATED)

At the moment, I think 'Linux' is the best Operating System around, for all cpu
types. And you pehraps ask me...why???
Well...in first place, it's all FREE!
Second, i think Linux has all features which an Advanced Operating System should
 have.
Third, Linux is very flexible when speaking about supporting new features.
But the Linux environment, is not so simple and practical as we would expect.
Should be a good environment for programmers! :)
Only experienced users know how to configure it and even then, in some situations 
is not so easy.
So, i had an idea of developping my dream Operating System.
My idea was to develop a new Operating System from scratch, nice, clean, easy, 
practical and at the same time, running with a small amount of code.
At the time, I am developping Heaven OS in NASM(The FREE Netwide 
Assembler) and testing it on Bochs (A FREE x86 CPU Emulator), under Linux, which is 
allways a good idea ! :)
An Object Oriented Compiler is on the way, to make coding even more easier.
All people can contribute on this project, with feedback for helpping 
solving some problems and programming applications, drivers or libs for the 
Operating System.


=============[ ACTUAL DESCRIPTION

This project focus on developping an original alternative 32-bit operating system for Intel 
80386 compatible processors.
Is not intended to compare it to the today modern operating systems, but
try to get the best features around and hope to discover better ways 
to do things.
At first, the operating system should be a simple and pratical plataform for developpment 
running with a small amount of code, after that, who knows what it could be.


============[ OPERATING SYSTEM INITIALIZATION STEPS (EXAMPLE)

----------------------------------------------------------------
 Boot Sector 512 Byte Image
-----------------------------------------------------------------

- Setting Registers for 16-Bit Segments
- Check for 80386+ CPU Family
- Show the Initialization message on the screen
- Copy Kernel from disk to a pre-defined location in memory

-----------------------------------------------------------------
HeavenOS Kernel Image
-----------------------------------------------------------------

- Disable interrupts ( we don't need them now! )
- Enable GATE A20 ( for Addressing Memory 1MB+ )
- Load the pre-defined GDT into GDTR(GDT Register)
- Switch to PMODE ( YEAH!! That's it! )
- ok, now make a FAR JUMP to the KERNEL 32-Bit location address
- Setting 32-Bit PMODE registers
- Initialize IDT with Interrupt Routine addresses
- Load the pre-defined IDT into IDTR(IDT Register)
- Programming the PIC8259 for IDT (Enabling only IRQ0,IRQ1,IRQ2 and IRQ8)
- Disabling IRQ0 (PIT interrupts)
- Programming the PIT8254 Chip (IRQ0)
- Programming the Keyboard 8042 Chip (IRQ1)
- Enabling RTC interrupts (IRQ8)
- Enable Interrupts
- Show the Initialization mensage on the screen
- Check CPU Family
- Check CPU Signature if possible
- Check CPU ID if possible
- Check System RAM
- Check System Date
- Check System Time
- Check PNP Bios
- Check PCI Devices(only S3 Cards, for testing!)
- Check ISA PNP Devices(only Creative Labs and OPTi Cards (address 0x200), for testing!)
- Some Delay (with RTC Chip)
- Fade Out Effect
- Initialize TSS's Information
- Load the pre-defined TSS Descriptor into TR(Task Register)
- Disable Interrupts
- Enabling IRQ0, Timer TASK GATE(for Sheduler)
- Jump to the Sheduler (begin TASK-SWITCH and enable interrupts)
  (MULTI-TASKING)
- TASK0: sheduler (IRQ0 - task manager)
- TASK1: shell
- TASK2: scroll mensage
- TASK3: time
- TASK4: date
-----------------------------------------------------------------
TODO:
-----------------------------------------------------------------
- Copy Floppy Disk Image to Ramdisk
- Do Filesystem Operations


===============[ LAST NEWS MESSAGE (OLD)

The development is really, really slow, if not stopped at all! :(
The cause of this..is a problem that i found on Boot Loader code..and til now dont know how to resolve it..
or what the problem really is.
SITUATION:

When Copying Kernel from Floppy Disk to Memory (INT 13h function 02h),
with kernel size greater than X bytes, the system enters in a loop!

Well..I hope someone have more luck than i, to solve the problem! :)
Against this..new features were added to the kernel and some code routines reviewed! :)

So..you can expect a new version of the Operating System when the Boot Loader code problem, 
is solved, and working well.

AT THE TIME, THE DEVELOPMENT WAS STOPPED! :(


=============[ DOWNLOADS

Operating System Source Code
http://heavenos.sourceforge.net/

BOCHS (A FREE PC x86 Emulator)
http://bochs.sourceforge.net/

NASM (The FREE Netwide Assembler)
http://nasm.sourceforge.net/

DJGPP - FREE 32-Bit C/C++ Compiler (DOS)
http://www.delorie.com/djgpp/

LCC - FREE 32-Bit C/C++ Compiler (Win32)
http://www.cs.virginia.edu/~lcc-win32/



=============[ TESTING

Writing Kernel Image to floppy disk under LINUX :

=>dd if=HeavenOS.img of=/dev/fd0 bs=512 seek=0");

Writing Kernel Image to floppy disk under DOS/WINDOWS :

=>rawrite -f HeavenOS.img -d a

When running the kernel:
-----------------------

you may need to press ENTER or ESC to continue the processing.. 


============[ THANKS TO SOME HACKERS OF WISDOM

This goes to those guys who help me with their knowledge and sometimes with code, other times with technical documents on chat conversations.


