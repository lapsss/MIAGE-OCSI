#ifndef LTR501_H
#define LTR501_H
#include <Wire.h>
unsigned int ADC_0, ADC_1, DIST;
float distance;

void WriteLight(unsigned char reg, unsigned char data){
  Wire.beginTransmission(0x23);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

unsigned char ReadLight(unsigned char reg){
  unsigned char data=0;
  
  Wire.beginTransmission(0x23);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(0x23, 1);
  while(Wire.available()){
    data = Wire.read();  
  }
  
  return data;
}
void initLightSensor() {
    Wire.begin();
  //  unsigned char id = 
  ReadLight(0x86);
  WriteLight(0x80, 0x03);  //Active mode, 64k lux range
  WriteLight(0x81, 0x03);  //PS active mode, x1 GAIN
  WriteLight(0x82, 0x6B);  //LED 60Hz, 50% duty, 50mA
  WriteLight(0x83, 0x7F);  //127 pulses
  WriteLight(0x84, 0x02);  //PS 100ms measure rate
  WriteLight(0x85, 0x03);  //ALS Integration 100ms, repeat rate 500ms
  // id
}

int getLighSensorMeasure() {
    int feedback=0;
    unsigned char stat = ReadLight(0x8C);
    if(stat & 0x04){
      //ALS new data
      unsigned char data[4];
      for(int i = 0; i < 4; i++){
        data[i] = ReadLight(0x88 + i);
      }
      ADC_1 = (data[1] << 8) | data[0];
      ADC_0 = (data[3] << 8) | data[2];
      feedback = ADC_1;
      //Serial.print("ALS1: ");
      //Serial.print(ADC_1, DEC);
      //Serial.println(" LUX");
      //Serial.print("ALS0: ");
      //Serial.print(ADC_0, DEC);
      //Serial.println(" LUX");
    
    }
    return feedback;
}
int getLighSensorDistance() {
        int feedback=0;
    unsigned char stat = ReadLight(0x8C);
    if(stat & 0x01){
      //PS new data
      unsigned char data[2];
      for(int i = 0; i < 2; i++){
        data[i] = ReadLight(0x8D + i);
      }
      DIST = (data[1] << 8) | data[0];
      distance = 10 -(10.0/2047)*DIST;
      feedback = distance;
    }   
    return feedback;
}
#endif