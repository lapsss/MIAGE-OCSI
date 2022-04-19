 /**
  * @file
  * @author Stefan Mavrodiev@OLIMEX LTD <support@olimex.com>
  * @version 1.0
  * 
  * @section LICENSE
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  * 
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  * 
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
  * MA 02110-1301, USA.
  * 
  * @section DESCRIPTION
  * 
  * Support library for Olimexino-328.
  */

#include "BMP085.h"
#include <Arduino.h>
#include <Wire.h>


uint8_t BMP085::address = 0x77;
uint8_t BMP085::oss = 1;
uint16_t BMP085::sleep = 7500;

uint32_t BMP085::temperature = 0;
uint32_t BMP085::pressure = 0;

int16_t BMP085::CONST_AC1 = 0;
int16_t BMP085::CONST_AC2 = 0;
int16_t BMP085::CONST_AC3 = 0;
uint16_t BMP085::CONST_AC4 = 0;
uint16_t BMP085::CONST_AC5 = 0;
uint16_t BMP085::CONST_AC6 = 0;        
int16_t BMP085::CONST_B1 = 0;
int16_t BMP085::CONST_B2 = 0;
int16_t BMP085::CONST_MB = 0;
int16_t BMP085::CONST_MC = 0;
int16_t BMP085::CONST_MD = 0;

long BMP085::X1 = 0;
long BMP085::X2 = 0;
long BMP085::B5 = 0;

/**
 * Constructor that sets device address.
 * 
 * @param Address of slave device.
 * @return Object to control MOD-BMP085.
 */
BMP085::BMP085(uint8_t addr){
    address = addr;
}
/**
 * Default constructor. The object will use the default address (0x77).
 * 
 * @param None
 * @return Object to control MOD-BMP085
 */
BMP085::BMP085(){
}
/**
 * Define oversampling
 * 
 * @param OSS: 0-ultra low power; 1-normal; 2-high resulotion; 3-ultra high resolution;
 * @return None.
 */
void BMP085::setOSS(uint8_t OSS){
    switch(OSS)
    {
        case 0:
            sleep = 4500;
            oss = 0;
            break;
        case 1:
            sleep = 7500;
            oss = 1;
            break;
        case 2:
            sleep = 13500;
            oss = 2;
            break;
        case 3:
            sleep = 25500;
            oss = 3;
            break;
        default:
            oss = 1;
            sleep = 7500;
            break;
    }
}
/**
 * Get current oversampling
 * 
 * @param None.
 * @return Current OSS.
 */
uint8_t BMP085::getOSS(){
    return oss;
}

void BMP085::loadConstants(){
    unsigned char data[22];
    unsigned char i = 0;
    Wire.begin();
    Wire.beginTransmission(address);
    Wire.write(0xAA);
    Wire.endTransmission();
    Wire.requestFrom((int)address, 22);
    while(Wire.available()){
        data[i++] = Wire.read();
    }
    
    CONST_AC1 = (data[0] << 8) | data[1];
	CONST_AC2 = (data[2] << 8) | data[3];
	CONST_AC3 = (data[4] << 8) | data[5];
	CONST_AC4 = (data[6] << 8) | data[7];
	CONST_AC5 = (data[8] << 8) | data[9];
	CONST_AC6 = (data[10] << 8) | data[11];
	CONST_B1 = (data[12] << 8) | data[13];
	CONST_B2 = (data[14] << 8) | data[15];
	CONST_MB = (data[16] << 8) | data[17];
	CONST_MC = (data[18] << 8) | data[19];
	CONST_MD = (data[20] << 8) | data[21];      
}

void BMP085::readMeasurement(){
    temperature = 0;
    pressure = 0;
    /**
     * Read temperature
     */
    Wire.begin();
    Wire.beginTransmission(address);
    Wire.write(0xF4);
    Wire.write(0x2E);
    Wire.endTransmission();
    delayMicroseconds(4500);
    
    Wire.beginTransmission(address);
    Wire.write(0xF6);
    Wire.endTransmission();
    Wire.requestFrom((int)address, 2);
    temperature = Wire.read();
    temperature <<= 8;
    temperature |= Wire.read();
    
    /**
     * Read pressure
     */
     Wire.begin();
     Wire.beginTransmission(address);
     Wire.write(0xF4);
     Wire.write(0x34+(oss<<6));
     Wire.endTransmission();
     delayMicroseconds(sleep);
     
     Wire.beginTransmission(address);
     Wire.write(0xF6);
     Wire.endTransmission();
     Wire.requestFrom((int)address, 3);
     pressure = Wire.read();
     pressure <<= 8;
     pressure |= Wire.read();
     pressure <<= 8;
     pressure |= Wire.read();
     pressure >>= (8-oss);
}
float BMP085::getTemperature(){
    
    X1 = (temperature - CONST_AC6) * CONST_AC5 / pow(2, 15);
    X2 = CONST_MC * pow(2, 11) / (X1 + CONST_MD);
    B5 = X1+X2;
    float T = ((B5 + 8)/pow(2, 4))/10.0;
    return T;
}

float BMP085::getPressure(){
    
    long B6 = B5 - 4000;
    X1 = (CONST_B2*(B6*B6/pow(2, 12)))/pow(2,11);
    X2 = CONST_AC2 * B6 / pow(2, 11);
    long X3 = X1 + X2;
    long B3 = (((CONST_AC1*4+X3)<<oss)+2)/4;
    X1 = CONST_AC3*B6/pow(2, 13);
    X2 = (CONST_B1*(B6*B6/pow(2,12)))/pow(2, 16);
    X3 = ((X1 + X2) + 2)/4;
    unsigned long B4 = CONST_AC4*(unsigned long)(X3 + 32768)/pow(2, 15);
    unsigned long B7 = ((unsigned long)pressure - B3)*(50000 >> oss);
    long p;
    if(B7 < 0x80000000)
        p = (B7*2)/B4;
    else
        p = (B7/B4)*2;
    X1 = (p/pow(2, 8))*(p/pow(2, 8));
    X1 = (X1 * 3038)/pow(2,16);
    X2 = (-7357*p)/pow(2, 16);
    p = p + (X1+X2+3791)/pow(2, 4);
    
    return p/100.0;
}
