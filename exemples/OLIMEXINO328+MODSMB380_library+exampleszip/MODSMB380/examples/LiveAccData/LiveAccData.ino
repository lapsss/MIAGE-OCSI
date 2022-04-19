/*
	Example code for
	Olimexino328 + MOD-SMB380
	
    Live raw acceleration data on the terminal
        	
    If you have any questions, email
    support@olimex.com
        
    OLIMEX, SEPTEMBER 2012 
	https://www.olimex.com/dev
*/

#include "MODSMB380.h"

// Definitions for the OlimexinoSTM32 UEXT SPI PORT
// If you have another board, refer to its manual
// for the correct definitions
#define CLOCK 13    // SPI Clock
#define CS    7     // Chip Select
#define MOSI  11    // Master OUT - Slave IN line
#define MISO  12    // Master IN  - Slave OUT line

#define BBIT (PIND & B00000100)!=0		// Used to check if the button has been pressed
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

	char ESC = 27;
	void ShowData();

void setup(){
	  Serial.begin(9600);
  
  //Press The Button to start the sketch
   WAIT;
  Serial.println("Olimexino-STM32 + MOD-SMB380 Example");
  Serial.println("");
  Serial.println("Live data (press the BUT button to start): ");        
  WAIT;  
}

void loop(){
  
  ShowData();
  delay(500);
}

void ShowData(){  
      smb380.updateData();
      
      Serial.print(ESC);
      Serial.print("[2J");
      Serial.print(ESC);
      Serial.print("[H");
      
      Serial.print("X : ");
      Serial.println(smb380.getAccX(), 2);
      Serial.print("Y : ");
      Serial.println(smb380.getAccY(), 2);
      Serial.print("Z : ");
      Serial.println(smb380.getAccZ(), 2);     
}