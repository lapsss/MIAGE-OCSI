/*
Utility Library for Thingworx Communication from ESP 32 Devices
Author : Bertrand Jauffret
*/

#ifndef __ThingworxRest_h
#define __ThingworxRest_h
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#endif

class ThingworxRest
{
public:
  ThingworxRest(String twxURL, String AppKey);
  int postToThing(String nameOfThing, String endPoint, String postBody);
  int httpPutPropertry(String thingName, String property, String value);
  String httpGetPropertry(String thingName, String property);
  bool httpGetBoolPropertry(String thingName, String property);
  String httpGetStringPropertry(String thingName, String property);
  void setDebug(bool in);
  String TWPlatformBaseURL;
  String APP_KEY;

private:
  boolean debug;

};

#endif // def(__TinyGPSPlus_h)

