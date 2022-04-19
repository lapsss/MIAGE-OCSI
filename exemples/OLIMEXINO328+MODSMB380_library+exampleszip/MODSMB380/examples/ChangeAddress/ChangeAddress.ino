/*
	Example code for
	Olimexino328 + MOD-SMB380
	
        Changes the value of a single address
        	
        If you have any questions, email
        support@olimex.com
        
        OLIMEX, SEPTEMBER 2012 
		https://www.olimex.com/dev
*/

#include "MODSMB380.h"

// Definitions for the Olimexino3238 UEXT SPI PORT
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
	Serial.println("Write a single Address (press button to start): ");
        
    WAIT;
    
    Serial.println("Customer-reserved address 1(0x12)");
    Serial.print(" Before writing: ");
    Serial.println(smb380.ReadOneAddr(0x12), 2);
    Serial.println(" Writing value B10101010...");
    smb380.WriteOneAddr(0x12, B10101010);
    Serial.print(" After writing: ");
    Serial.println(smb380.ReadOneAddr(0x12), 2);
    Serial.print("");
    Serial.print("Done!");     
        
}

void loop()
{}
        
        
        