/*
Utility Library for Thingworx Communication from ESP 32 Devices
Author : Bertrand Jauffret
*/
#include "ThingworxRest.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#define ACCEPT_TYPE "application/json"

//Class  Constructor
ThingworxRest::ThingworxRest(String twxURL, String AppKey) {
 debug = true;
 APP_KEY = AppKey;
 TWPlatformBaseURL = twxURL;
}
// Methods

void ThingworxRest::setDebug(bool in){
  debug = in;
}
///////////////////////////////
// make HTTP GET to a specific Thing and Propertry on a ThingWorx server
// thingName - Name of Thing on server to make GET from
// property - Property of thingName to make GET from
// returns full HTTP Response as String
///////////////////////////////
String ThingworxRest::httpGetPropertry(String thingName, String property) {
  if (debug) Serial.println("[httpGetPropertry] begin");
  
  HTTPClient https;
  int httpCode = -1;
  String response = "";
  if (debug) Serial.print("[httpsGetPropertry] begin... | ");
  String fullRequestURL = String(TWPlatformBaseURL) + "/Thingworx/Things/" + thingName + "/Properties/" + property ;
  https.begin(fullRequestURL);
  https.addHeader("Accept", ACCEPT_TYPE, false, false);
  https.addHeader("appKey", APP_KEY, false, false);
  if (debug) Serial.println("GET URL>" + fullRequestURL + "<");
  // start connection and send HTTP header
  httpCode = https.GET();
  if (debug) Serial.println("[httpGetPropertry] GET URL>" + fullRequestURL);
  // httpCode will be negative on error
  if (httpCode > 0) {
    response = https.getString();
    if (debug) Serial.printf("[httpGetPropertry] response code:%d body>", httpCode);
    if (debug) Serial.println(response + "<\n");
  } else {
    Serial.printf("[httpGetPropertry] failed, error: %s\n\n", https.errorToString(httpCode).c_str());
  }

  if (debug) Serial.println("[httpGetPropertry] end");
  https.end();
  return response;

}

///////////////////////////////
// make HTTP GET to a specific Thing and BOOLEAN Propertry on the  ThingWorx server
// thingName - Name of Thing on server to make GET from
// property - Property of thingName to make GET from
// returns the associated boolean value
///////////////////////////////

bool ThingworxRest::httpGetBoolPropertry(String thingname, String property) {
  // Fetch the property from the server :
 String json =  httpGetPropertry(thingname,property);
DynamicJsonDocument doc(1024);
deserializeJson(doc, json);
bool output = doc["rows"][0][property];
if (debug) Serial.println("Configuration " + property + " : " + output);
return output;
}

String ThingworxRest::httpGetStringPropertry(String thingname, String property) {
  // Fetch the property from the server :
 String json =  httpGetPropertry(thingname,property);
DynamicJsonDocument doc(1024);
deserializeJson(doc, json);
String output = doc["rows"][0][property];
if (debug) Serial.println("Configuration " + property + " : " + output);
return output;
}


///////////////////////////////
// make HTTP PUT to a specific Thing and Propertry on a ThingWorx server
// thingName - Name of Thing on server to execute the PUT
// property - Property of thingName to make PUT request
// returns HTTP response code from server and prints full response
///////////////////////////////
int ThingworxRest::httpPutPropertry(String thingName, String property, String value) {
  if (debug) Serial.println("[httpPutPropertry] begin");
  
  HTTPClient httpClient;
  int httpCode = -1;
  String response = "";  
  String fullRequestURL = String(TWPlatformBaseURL) + "/Thingworx/Things/" + thingName + "/Properties/" + property ; //+"?appKey=" + String(appKey);
  httpClient.begin(fullRequestURL);
  httpClient.addHeader("Accept", ACCEPT_TYPE, false, false);
  httpClient.addHeader("Content-Type", ACCEPT_TYPE, false, false);
  httpClient.addHeader("appKey", APP_KEY, false, false);
  
  // start connection and send HTTP header
  String putBody = "{\"" + property + "\":" + value + "}";
  
  if (debug) Serial.println("[httpPutPropertry] VALUES : " + putBody );
  if (debug) Serial.println("[httpPutPropertry] PUT URL>" + fullRequestURL);
  
  httpCode = httpClient.PUT(putBody);
  
  // httpCode will be negative on error
  if (httpCode > 0) {
    response = httpClient.getString();
    if (debug) Serial.printf("[httpPutPropertry] response code : %d | body : ", httpCode);
    if (debug) Serial.println(response + "<");
  } else {
    Serial.printf("\n[httpPutPropertry] failed, error: %s\n\n", httpClient.errorToString(httpCode).c_str());
  }
  httpClient.end();
  if (debug) Serial.println("[httpPutPropertry] end\n.");
  return httpCode;
}


///////////////////////////////
// make HTTP POST to ThingWorx server Thing service
// nameOfThing - Name of Thing to POST to
// endPoint - Services URL to invoke
// postBody - Body of POST to send to ThingWorx platform
// returns HTTP response code from server
///////////////////////////////
int ThingworxRest::postToThing(String nameOfThing, String endPoint, String postBody) {
  if (debug) Serial.println("[postToThing] begin");
  
  HTTPClient https;
  int httpCode = -1;
  String response = "";
  String fullRequestURL = String(TWPlatformBaseURL) + "/Thingworx/Things/" + nameOfThing + "/Services/" + endPoint;
  if (debug) Serial.println("URL>" + fullRequestURL + "<");
  https.begin(fullRequestURL);
  https.addHeader("Accept", "application/json", false, false);
  https.addHeader("Content-Type", "application/json", false, false);
  https.addHeader("appKey", APP_KEY, false, false);
   
  if (debug) Serial.println("[postToThing] VALUES : " + postBody );
  if (debug) Serial.println("[postToThing] URL>" + fullRequestURL);
  
  // start connection and send HTTP header
  httpCode = https.POST(postBody);
  
  // httpCode will be negative on error
  if (httpCode > 0) {
    response = https.getString();
    if (debug) Serial.printf("[postToThing] response code:%d body>", httpCode);
    if (debug) Serial.println(response + "<\n");

  } else {
    Serial.printf("[postToThing] POST... failed, error: %s\n\n", https.errorToString(httpCode).c_str());
  }
  https.end();
  if (debug) Serial.println("[httpPutPropertry] end\n.");
  return httpCode;
}
