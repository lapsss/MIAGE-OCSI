|----------------------------------------------------------------------|
|	 README for the MODSMB380 project for the ArduinoIDE   			   |
|----------------------------------------------------------------------|
      
	Arduino IDE version 1.0.1
						   
	This library was tested with an Olimexino-328 + MODSMB380
	
		* MOD-SMB380 - a board with a BOSCH SMB380 Triaxial 
					   acceleration sensor
	
	How to install the library:
	1) Make sure that the Arduino IDE is not running
	2) Copy the MODSMB380 folder to the /libraries folder of
	   your Maple IDE installation folder
	3) Start the IDE. The "MODSMB380" examples should now
		appear under File/examples.
	4) Include the library in your projects as such:
		#include "MODSMB.h"		
		
	The library provides three examples that utilize the MODSMB380
	class interface:
	
	- Temperature - Continuously prints the current temperature in C
					on the terminal
	- ChangeAddress - Uses the WriteOneAddr function to set the value of
					  one customer-reserved address. Note, that the value
					  of the address must be between 0 and 0x15 (==15h)
					  If the address exceeds 0x15, the chip SDI will go into
					  Tri-State mode enabling the communication of a second
					  device on the same CSD and SDI line.
	- LiveAccData   - A sketch that continuously updates the raw X, Y and Z
					  acceleration data (10 bits each) and writes it on the 
					  terminal, clearing it with each update, so that it doesn't
					  get cluttered.
					  
	There are useful notes before each function's code in the MODSMB380.h file.
	
	If you have any questions, email
	support@olimex.com
	
	OLIMEX, SEPTEMBER 2012
	https://www.olimex.com
	
	