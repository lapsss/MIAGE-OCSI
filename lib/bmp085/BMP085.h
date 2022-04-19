#ifndef BMP085_H
#define BMP085_H

#include <inttypes.h>
#include "Arduino.h"

class BMP085
{
    private:
        static uint8_t address;
        static uint8_t oss;
        static uint16_t sleep;
        
        static uint32_t temperature;
        static uint32_t pressure;
        
        /**
         * EEPROM constants
         */
        static int16_t CONST_AC1;
        static int16_t CONST_AC2;
        static int16_t CONST_AC3;
        static uint16_t CONST_AC4;
        static uint16_t CONST_AC5;
        static uint16_t CONST_AC6;        
        static int16_t CONST_B1;
        static int16_t CONST_B2;
        static int16_t CONST_MB;
        static int16_t CONST_MC;
        static int16_t CONST_MD;
        
        static long X1;
        static long X2;
        static long B5;

        
    public:
        BMP085();
        BMP085(uint8_t);
        
        void loadConstants();
        void setOSS(uint8_t);
        uint8_t getOSS();
        void readMeasurement();
        float getTemperature();
        float getPressure();

};
#endif
