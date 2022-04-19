/*
	Example code for
	Olimexino328 + MOD-SMB380
	
	Updates the temperature with an
	interrupt routine and continuously prints
	it on the terminal
	
        If you have any questions, email
        support@olimex.com
        
        OLIMEX, SEPTEMBER 2012 
		https://www.olimex.com/dev
*/

#include "MODSMB380.h"

// Definitions for the Olimexino328 UEXT SPI PORT
// If you have another board, refer to its manual
// for the correct definitions
#define CLOCK 13    // SPI Clock
#define CS    7     // Chip Select
#define MOSI  11    // Master OUT - Slave IN line
#define MISO  12    // Master IN  - Slave OUT line

#define BBIT (PIND & B00000100)!=0			// Used to check if button has been pressed
#define BUTTONINPUT DDRD &= B11111011		// Init the Button
#define YLED 9 
#define GLED 13

/**************************************************/
/** Code that waits for the user to press the button
**/	
	void wait_ch(){ 
		delay(500);
		while( BBIT ){}
		delay(300);	
		}
	
	#define WAIT wait_ch()
/**************************************************/

MODSMB380 smb380(CLOCK, CS, MOSI, MISO);

void setup(){ 
    Serial.begin(9600);
 
   // Press the BUT buton on the board to start the sketch...
    WAIT;
	
    Serial.println("Olimexino-STM32 + MOD-SMB380 Example");
	Serial.println("");
	Serial.println("Current Temperature: ");
}

void loop(){	
        smb380.updateData();
		
		// The module is accurate up to .5 degrees
		Serial.println( smb380.getTemp(), 1);		// print up to one digit after the decimal point 
		delay(500);	
}
