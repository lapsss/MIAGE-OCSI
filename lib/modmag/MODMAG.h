/*
	MOD-MAG Library for the Arduino IDE 1.0.3
	
	Uses software I2C communication
	
	This library has been tested with
	Olimexino-328, but should work with any Arduino
	board, which has an I2C interface.
	
	If you have any questions, email
	support@olimex.com
	
	https://www.olimex.com
	OLIMEX 2013
	
	  This library is free software; you can redistribute it and/or
	  modify it under the terms of the GNU Lesser General Public
	  License as published by the Free Software Foundation; either
	  version 2.1 of the License, or (at your option) any later version.

	  This library is distributed in the hope that it will be useful,
	  but WITHOUT ANY WARRANTY; without even the implied warranty of
	  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	  Lesser General Public License for more details.
	
 */	

	

#ifndef MODMAG_H
#define MODMAG_H

#include "Arduino.h"
	
class MODMAG
{
private:
	static MODMAG * currentObj;
	unsigned char pinSDA, pinSCL;
	const static char  ACK = 0;
	const static char NACK = 1;
	const static unsigned char RW_MASK  = 0x80;
	const static unsigned char SL_A     = 0x0E;
	const static unsigned char SL_READ  = 0x1D;
	const static unsigned char SL_WRITE = 0x1C;
	
	const static unsigned char CTRL_REG1 = 0x10;
	const static unsigned char CTRL_REG2 = 0x11;
	
	short int X, Y, Z;
	int8_t Temperature;
		
public:
	MODMAG( unsigned char nSDA = SDA, unsigned char nSCL = SCL )
			:pinSDA(nSDA), pinSCL(nSCL){ 
									MODMAG::currentObj = this; 
									SCL_OUT();
									SDA_OUT();
								 }
	~MODMAG(){}
	
	void SDA_OUT(){ pinMode(pinSDA, OUTPUT);}
	void SDA_IN() { pinMode(pinSDA,  INPUT);}
	void SCL_OUT(){ pinMode(pinSCL, OUTPUT);}
	void SCL_IN() { pinMode(pinSCL, INPUT); } 
	
	void Start(){
			SDA_OUT();
			WaitMicrosecond(1000);
			SCL_OUT();
			WaitMicrosecond(1000);		
	}
		
	void Stop(){
			SDA_OUT();
			WaitMicrosecond(1000);
			SCL_IN();
			WaitMicrosecond(1000);
			SDA_IN();
			WaitMicrosecond(1000);
	}
		
	char WriteSingle( char data ){
			char i;
			
			// Send the 8 bits
			for(i = 0; i<8; i++)
			{
				WaitMicrosecond(1000);
				if(data&RW_MASK) SDA_IN(); else SDA_OUT();
				data <<= 1;
				WaitMicrosecond(1000);
				SCL_IN();
				WaitMicrosecond(1000);
				SCL_OUT();
			}
		
			// Read the ACK
			WaitMicrosecond(1000);
			SDA_IN();
			WaitMicrosecond(1000);
			SCL_IN();
			WaitMicrosecond(1000);
			i = digitalRead(pinSDA);
			SCL_OUT();
			WaitMicrosecond(1000);

			return i;		
	}
		
	char WriteSingleIadr(unsigned char W_ADDR, char data){
		
		char aflag;
		Start();
		aflag = WriteSingle(SL_WRITE);
		aflag|= WriteSingle (W_ADDR);
		aflag|= WriteSingle (data);
		Stop();
		delay(500);
		
		return !aflag;
		
	}
	
	char WriteMultipleIadr(unsigned char W_ADDR, char * data, unsigned char NumOfBytes){
		
		char aflag;
		Start();
		for(int i = 0; i<NumOfBytes; i++)
		{
			aflag = WriteSingle (W_ADDR);
			aflag|= WriteSingle (data[i]);
			W_ADDR++;
		}
		Stop();
		delay(500);
		
		return !aflag;		
	
	}
	
	char ReadSingle( char ack ){
		
		char data = 0;
		char i;
		
		SDA_IN();		
		for(i = 0; i < 8; i++)
		{
			WaitMicrosecond(1000);
			SCL_IN();
			while(digitalRead(pinSCL)==0);
			WaitMicrosecond(1000);
			data |= digitalRead(pinSDA);
			if( i!=7 )
				data<<=1;
			WaitMicrosecond(1000);
			SCL_OUT();
			WaitMicrosecond(1000);
		}
		
		// send the ACK/NACK
		WaitMicrosecond(1000);
		if(ack) SDA_IN(); else SDA_OUT();
		WaitMicrosecond(1000);
		SCL_IN();
		WaitMicrosecond(1000);
		SCL_OUT();
		WaitMicrosecond(1000);

		return data;		
	}
	
	char ReadSingleIadr(unsigned char R_ADDR, unsigned char &aPtr){
		unsigned char aflag;
		char data;
		
		Start();
		aflag = WriteSingle(SL_WRITE);
		aflag|= WriteSingle(R_ADDR);
		Stop();
		Start();
		aflag|= WriteSingle(SL_READ);
		data = ReadSingle(NACK);
		Stop();
		
		aPtr = aflag;
		
		return data;
			
	}
	
	char ReadMultipleIadr(unsigned char R_ADDR, char * buffer, unsigned char NumOfBytes){
		
		unsigned char aflag;
		if(NumOfBytes==1)
		{
			buffer[0] = ReadSingleIadr(R_ADDR, aflag);
			return (!aflag);
		}
		else
		{
			Start();
			aflag = WriteSingle(SL_WRITE);
			aflag|= WriteSingle(R_ADDR);
			Stop();
			Start();
			aflag|= WriteSingle(SL_READ);
			for(int i=0; i<NumOfBytes-1; i++)
				buffer[i] = ReadSingle(ACK);
				
			buffer[NumOfBytes-1] = ReadSingle(NACK);				
			Stop();
			
			return (!aflag);
		}
	
	}
	
	uint16_t updateData(){
		unsigned char flag;
		
		//Trigger Measurement by setting the TM bit in CTRL_REG1
		//Refer to page 12 and 17 of the MAG3110 manual for detailed information
		WriteSingleIadr(CTRL_REG2, 0x80);
		WriteSingleIadr(CTRL_REG1, 0b00011010);
		
		//Burst-read of the 6 bytes of data for X, Y and Z
		char buffer[6]={0};
		while(!(ReadMultipleIadr(0x01, buffer, 6)));
		X = buffer[0]; X<<=8; X |= buffer[1];
		Y = buffer[2]; Y<<=8; Y |= buffer[3];
		Z = buffer[4]; Z<<=8; Z |= buffer[5];	
		
		WaitMicrosecond(2000);
		Temperature = ReadSingleIadr(0x0f, flag);
		return X*Y*Z;
	}
	
	int8_t getTemperature(){ return Temperature; }	
	short int  getX() {return X;}
	short int  getY() {return Y;}
	short int  getZ() {return Z;}
	
	void WaitMicrosecond(int x){ delayMicroseconds(x); }	
	
	unsigned char getSDAnum(){ return pinSDA; }
	unsigned char getSCLnum(){ return pinSCL; }
};

MODMAG * MODMAG::currentObj = NULL;

#endif