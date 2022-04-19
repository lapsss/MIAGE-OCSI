/*
	Library for the Olimex MOD-SMB380 board and Arduino Boards
	SMB380 - Triaxial acceleration sensor 
	
	This Library Uses Software SPI Communication
	It Reads/Writes data to the SMB380 chip via SPI
	The class provides some useful Interface functions
	
	Tested with Olimexino-328 and MOD-SMB380
	
	|-------------------
	| v1.0 - September 2012, Arduino IDE 1.0.1
	|-------------------
	
	For more information, visit:
	https://www.olimex.com/dev/
	
	If you have any questions, email:
	support@olimex.com
	
	  This library is free software; you can redistribute it and/or
	  modify it under the terms of the GNU Lesser General Public
	  License as published by the Free Software Foundation; either
	  version 2.1 of the License, or (at your option) any later version.

	  This library is distributed in the hope that it will be useful,
	  but WITHOUT ANY WARRANTY; without even the implied warranty of
	  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	  Lesser General Public License for more details.
	  
	  Olimex, September 2012
	  https://www.olimex.com
	  	  	
*/

#ifndef MODSMB380_H
#define MODSMB380_H

//#include "HardwareSPI.h"
#include "inttypes.h"
#include "Arduino.h"

class MODSMB380
{
private:
	volatile uint16_t accX, accY, accZ;
	volatile double Temperature;
	char CLOCK, CS, MOSI, MISO;
	
	static const unsigned char MASK_7 = 0x80;
	static const unsigned char DELAY_MICROS = 100;
	
   /** SPI Utility Functions **/   
   void CS_OUT(){ pinMode(CS, OUTPUT); }
   void CS_IN(){  pinMode(CS,  INPUT); }
   
   void CLOCK_HIGH(){ digitalWrite(CLOCK, HIGH); }
   void  CLOCK_LOW(){ digitalWrite(CLOCK, LOW); }
   
   void MOSI_1(){ digitalWrite(MOSI, HIGH); }
   void MOSI_0(){ digitalWrite(MOSI, LOW); }
   
   unsigned char GET_MISO(){ return digitalRead(MISO); }
   
   void SPI_DELAY(){ delayMicroseconds(DELAY_MICROS); }
	
public:

	/**
	 *  Constructor
	 */
	MODSMB380(char nClock, char nCS, char nMosi, char nMiso)
			:CLOCK(nClock), CS(nCS), MOSI(nMosi), MISO(nMiso){
			
			pinMode(CLOCK, OUTPUT);		// Set the clock line as output
			pinMode(MOSI, OUTPUT);		// Set the line for sending data as OUTPUT
			pinMode(MISO, INPUT);		// Set the line for receiving data as INPUT
			pinMode(CS, INPUT); 		// SPI is IDLE						
			}
	
	/** 
	 *	Destructor
	 **/
   ~MODSMB380(){}   
  
   /*** Class Interface ***/
   
	/** 
	 *	Writes a value to a single address 
	 *	W_ADDR = The address you want to write
	 *	data   = The 8 bits that you wantto write to W_ADDR
	 *
	 *	  Note, that the value
	 *	  of the address must be between 0 and 0x15 (==15h)
	 *	  If the address exceeds 0x15, the chip SDI will go into
	 * 	  Tri-State mode enabling the communication of a second
	 * 	  device on the same CSD and SDI line.
	 **/
	void WriteOneAddr(unsigned char W_ADDR, uint8_t data){
	
		char i;
		
		CS_OUT();					// Chip Select LOW, Start communication
		SPI_DELAY();				// Delay 100 microseconds 
		
		for(i=0; i<8; i++)
		{			
			if(W_ADDR & MASK_7)
			{	
				CLOCK_LOW();
				MOSI_1();
			}
			else
			{
				CLOCK_LOW();
				MOSI_0();
			}
				
			W_ADDR<<=1;
			
			SPI_DELAY();
				
			CLOCK_HIGH();			
		
			SPI_DELAY();
			SPI_DELAY();
		}
		
		for(i=0; i<8; i++)
		{			
			if(data & MASK_7)
			{	
				CLOCK_LOW();
				MOSI_1();
			}
			else
			{
				CLOCK_LOW();
				MOSI_0();
			}
				
			data<<=1;
			
			SPI_DELAY();
				
			CLOCK_HIGH();			
		
			SPI_DELAY();
			SPI_DELAY();
		}
		
		SPI_DELAY();
		CS_IN();						//Chip Select HIGH, SPI is Idle
		
	}
	
	
	/** 
	 *	Reads multiple addresses (00h to 15h)	 
	 *	R_ADDR = The first address you want to read
	 *	bufer  = The array where the values of successive addresses will be written
	 *	NumOfBytes  = The number of successive bytes you want to read
	 
	 *	  Note, that the value
	 *	  of the address must be between 0 and 0x15 (==15h)
	 *	  If the address exceeds 0x15, the chip SDI will go into
	 * 	  Tri-State mode enabling the communication of a second
	 * 	  device on the same CSD and SDI line.
	 **/	
	void ReadAddr(unsigned char R_ADDR, uint8_t * buffer, uint8_t NumOfBytes){
		
		char i;
		
		// Send the Address of the Register
		R_ADDR |= MASK_7;
		
		CS_OUT();					// Chip Select LOW, Start communication
		SPI_DELAY();				// Delay 100 microseconds 
		
		for(i=0; i<8; i++)
		{			
			if(R_ADDR & MASK_7)
			{	
				CLOCK_LOW();
				MOSI_1();
			}
			else
			{
				CLOCK_LOW();
				MOSI_0();
			}
				
			R_ADDR<<=1;
			
			SPI_DELAY();
				
			CLOCK_HIGH();			
		
			SPI_DELAY();
			SPI_DELAY();
		}
		
		MOSI_0();
		SPI_DELAY();
		SPI_DELAY();
		
		//Read the several successive bytes
		for(int j=0; j<NumOfBytes; j++)
		{
			// Read a single byte from the CHIP
			for(i=0; i<8; i++)
			{
				CLOCK_LOW();
				
				buffer[j] |= GET_MISO();
				if(i!=7) buffer[j] <<=1;
				
				SPI_DELAY();
				
				CLOCK_HIGH();
							
				SPI_DELAY();
				SPI_DELAY();		
			}					
		}
		
		SPI_DELAY();
		CS_IN();						//Chip Select HIGH, SPI is Idle
	}
	
	
	/** 
	 *	Reads a single address (00h to 15h)	 
	 *	R_ADDR = The address you want to read
	 
	 *	  Note, that the value
	 *	  of the address must be between 0 and 0x15 (==15h)
	 *	  If the address exceeds 0x15, the chip SDI will go into
	 * 	  Tri-State mode enabling the communication of a second
	 * 	  device on the same CSD and SDI line.
	 *	
	 **/
	uint8_t ReadOneAddr(unsigned char R_ADDR){
		unsigned char res=0;
		char i=0;
		
		// Send the Address of the Register
		R_ADDR |= MASK_7;
		
		CS_OUT();					// Chip Select LOW, Start communication
		SPI_DELAY();				// Delay 100 microseconds 
		
		for(i=0; i<8; i++)
		{			
			if(R_ADDR & MASK_7)
			{	
				CLOCK_LOW();
				MOSI_1();
			}
			else
			{
				CLOCK_LOW();
				MOSI_0();
			}
				
			R_ADDR<<=1;
			
			SPI_DELAY();
				
			CLOCK_HIGH();			
		
			SPI_DELAY();
			SPI_DELAY();
		}
		
		MOSI_0();
		SPI_DELAY();
		SPI_DELAY();
		
		// Read a single byte from the CHIP
		for(i=0; i<8; i++)
		{
			CLOCK_LOW();
			
			res |= GET_MISO();
			if(i!=7) res <<=1;
			
			SPI_DELAY();
			
			CLOCK_HIGH();
						
			SPI_DELAY();
			SPI_DELAY();		
		}
		
		SPI_DELAY();
		CS_IN();					//Chip Select HIGH, SPI is Idle
				
		return res;
	}
	
	
	/** 
	 *	Updates the variables with new data 
	 **/
	void updateData(){
		uint16_t rawX=0, rawY=0, rawZ=0, rawT=0;
		uint8_t  buf[6]={0};
		char lsb = 0;
		
		ReadAddr(0x02, buf, 6);
		
		rawX = buf[1]; rawX <<= 2; lsb = buf[0] & B11000000;
		rawX |= (lsb>>6);
		accX = rawX;		
		
		rawY = buf[3]; rawY <<= 2; lsb = buf[2] & B11000000;
		rawY |= (lsb>>6);
		accY = rawY;
		
		rawZ = buf[5]; rawZ <<= 2; lsb = buf[4] & B11000000;
		rawZ |= (lsb>>6);
		accZ = rawZ;
		
		rawT = ReadOneAddr(0x08);
		Temperature = (rawT>>1) - 30;
		if(rawT&0x01) Temperature = Temperature + 0.5;
		
	}
	
	
	/** 
	 *	Returns the X acceleration data (10 bits) 
	 **/	 
	uint16_t getAccX(){ return accX;}
	
	
	/** 
	 *	Returns the Y acceleration data (10 bits) 
	 **/
	uint16_t getAccY(){ return accY;}
	
	
	/** 
	 *	Returns the Z acceleration data (10 bits) 
	 **/
	uint16_t getAccZ(){ return accZ;}
	
	
	/** 
	 *	Returns Temperature data (8 bits) 
	 **/
	double  getTemp(){ return Temperature; }
	
	
	/** 
	 *	Returns the X, Y and Z acceleration data (10 bits each) 
	 *			using references
	 **/
	void getAcc( uint16_t &X, uint16_t &Y, uint16_t &Z){
		X = getAccX();
		Y = getAccY();
		Z = getAccZ();
	}
	
	
	/** 
	 *	Returns the number of the CS pin 
	 **/
	char pinCS(){ return CS;}
	
	
	/** 
	 *	Returns the number of the CLOCK pin 
	 **/
	char pinCLOCK(){ return CLOCK;}
	
	
	/** 
	 *	Returns the number of the MISO pin 
	 **/
	char pinMISO(){ return MISO;}
	
	
	/** 
	 *	Returns the number of the MOSI pin 
	 **/
	char pinMOSI(){ return MOSI;}
	
};
#endif