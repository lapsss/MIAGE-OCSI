/**
  Qualit'Air Project
  AMU - MIAGE 2021
   Draft example to be completed during class
*/

/* Dependent libraires: see install & setup document */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Adafruit_CCS811.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ThingworxRest.h>
#include <lcd3310.h>
#include <MODMAG.h>
#include <WebServer.h>
#include <ltr501.h>

//////////////////////////
// WiFi Configurations //
//  TO CHANGE         //
///////////////////////
const char WiFiSSID[] = "xxxxxxxxxxx"; // WiFi access point SSID
const char WiFiPSK[] = "xxxxxxxxxxx"; // WiFi password - empty string for open access points
//////////////////////////////////////////////
// ThingWorx server definitions            //
//  TO CHANGE                             //
///////////////////////////////////////////
String TWPlatformBaseURL = "xxxxxxxxxxx";
String APP_KEY = "xxxxxxxxxxxx";
const int INTERVAL = 10; //refresh interval
const char THING_PREFIX[] = "MIAGE.";
const char CO2_PROPERTY[] = "CO2";                 //Thing property name
const char TVOC_PROPERTY[] = "AirQuality";         //Thing property name
const char HUMIDITY_PROPERTY[] = "Humidity";       //Thing property name
const char TEMPERATURE_PROPERTY[] = "Temperature"; //Thing property name
const char PRESSURE_PROPERTY[] = "Pressure";       //Thing property name
ThingworxRest thingworx(TWPlatformBaseURL,APP_KEY);
//////////////////////////////////////////////
// Program execution settings              //
//  TO CHANGE                             //
///////////////////////////////////////////
bool debug=true;
bool info=true;
bool SENSOR_WEATHER=false; // Weather Sensors
bool SENSOR_GPS=false;
bool SENSOR_LCD=false;
bool SENSOR_MAG=false;
bool SENSOR_LIGHT=false;
//////////////////////////////////////////////////////////
// Pin Definitions - board specific for Adafruit board //
////////////////////////////////////////////////////////
Adafruit_CCS811 ccs;
Adafruit_BME280 bme; // I2C
const int RED_LED = 0; // Thing's onboard, red LED -
const int BLUE_LED = 2; // Thing's onboard, blue LED
const int ANALOG_PIN = 0; // The only analog pin on the Thing
const int OFF = HIGH;
const int ON = LOW;
// this will set as the Accept header for all the HTTP requests to the ThingWorx server
// valid values are: application/json, text/xml, text/csv, text/html (default)

///////////////////
//MOD GPS SETUP : all moved to the utility Class
/////////////////
unsigned long start;
HardwareSerial gpsSerial(Serial1);
#define BAUDRATE 9600 // this is the default baudrate of the GPS module
TinyGPSPlus gps;


// Mod Magnetometer
MODMAG modmag;

void sendMagData(String thingname){
  unsigned char aflag;
  //Serial.print("CHIP ID: ");
  //uint8_t chip_id = modmag.ReadSingleIadr(0x07, aflag);
  //Serial.println(chip_id, BIN);
  
  //Update values to current data
  //The MODMAG.h library uses the "Trigger Measurement" mode
  //Refer to the MAG3110 manual for detailed information
  //pages 12 and 17
  modmag.updateData();
  if (debug) Serial.println("Raw Data:");
  if (debug)  {Serial.print("X: "); Serial.println(modmag.getX(), DEC);}
  thingworx.httpPutPropertry(thingname,"MAG_X",String(modmag.getX()));
  if (debug) {Serial.print("Y: "); Serial.println(modmag.getY(), DEC);}
  thingworx.httpPutPropertry(thingname,"MAG_Y",String(modmag.getY()));
  if (debug) {Serial.print("Z: "); Serial.println(modmag.getZ(), DEC);}
  thingworx.httpPutPropertry(thingname,"MAG_Z",String(modmag.getZ()));
  if (debug) { Serial.print("Temperature: "); Serial.println(modmag.getTemperature(), DEC);}
  thingworx.httpPutPropertry(thingname,"MAG_TEMP",String(modmag.getTemperature()));
}

// Send the Light Data
void sendLightData(String thingname) {
  int lux = getLighSensorMeasure();
  int distance = getLighSensorDistance();
  
  thingworx.httpPutPropertry(thingname,"LIGHT_LUX",String(lux));
  thingworx.httpPutPropertry(thingname,"LIGHT_DIST",String(distance));
  
}

/////////////////////
// WiFi connection. Checks if connection has been made once per second until timeout is reached
// returns TRUE if successful or FALSE if timed out
/////////////////////
boolean connectToWiFi(int timeout) {

  Serial.println("Connecting to: " + String(WiFiSSID));
  WiFi.begin(WiFiSSID, WiFiPSK);

  // loop while WiFi is not connected waiting one second between checks
  uint8_t tries = 0; // counter for how many times we have checked
  while ((WiFi.status() != WL_CONNECTED) && (tries < timeout) ) { // stop checking if connection has been made OR we have timed out
    tries++;
    Serial.printf(".");// print . for progress bar
    Serial.println(WiFi.status());
    delay(2000);
  }
  Serial.println("*"); //visual indication that board is connected or timeout

  if (WiFi.status() == WL_CONNECTED) { //check that WiFi is connected, print status and device IP address before returning
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else { //if WiFi is not connected we must have exceeded WiFi connection timeout
    return false;
  }

}

//////////////////////////////
//create a name for the board that is unique. Manual input
//return name as a String
///////////////////////////////
String getUniqueDeviceName() {

  String mac = WiFi.macAddress();
  mac.replace(":","");
  Serial.println("DeviceID>" + mac);
  return String(THING_PREFIX + mac);
}

 // Init Debug Output to tty
void initBoard() {
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  delay(2000);
  Serial.flush();
  Serial.println("Starting Firmware...");
  Serial.println("Done!");
  Serial.printf("MOSI : ");Serial.println(MOSI);
  Serial.printf("MISO : ");Serial.println(MISO);
  Serial.printf("SCK : ");Serial.println(SCK);
  Serial.printf("SS : ");Serial.println(SS);
// Connect  Wifi  
  connectToWiFi(10);
  //Serial.println(RST);
  Serial.flush();
  // Fetch configuration from the Thingworx Server
  String thingname = getUniqueDeviceName();
  // Fetch configuration properties from the server :
  debug = thingworx.httpGetBoolPropertry(thingname,"DEBUG_DEVICE");
  thingworx.setDebug(debug);
  info = thingworx.httpGetBoolPropertry(thingname,"INFO_DEVICE");
  SENSOR_MAG = thingworx.httpGetBoolPropertry(thingname,"SENSOR_MAG"); 
  SENSOR_GPS= thingworx.httpGetBoolPropertry(thingname,"SENSOR_GPS"); 
  SENSOR_LCD= thingworx.httpGetBoolPropertry(thingname,"SENSOR_LCD"); 
  SENSOR_WEATHER= thingworx.httpGetBoolPropertry(thingname,"SENSOR_WEATHER"); 
  SENSOR_LIGHT = thingworx.httpGetBoolPropertry(thingname,"SENSOR_LIGHT"); 

}

// Init Weather Sensors (BME & CCS)
void initWeather() {
  if (!ccs.begin()) {
    Serial.println("BME Sensor not Started");
  }
  // Wait for the sensor to be ready
  while (!ccs.available());
  bme.begin();
}

  // Init GPS :
void initGps() {  
  byte settingsArray[] = {0x03, 0xFA, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
  gps.configureUblox(settingsArray,gpsSerial);
}



void displayTest() {
    //  LCDStr(0, (unsigned char *) "MIAGE", 0);
    LCDTriangle(10,10,80,45,0,47);
    LCDCircle(50,30,10);
    LCDRectangle(20,20,40,40);
    LCDUpdate();
    
  }

void initLCD() {
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  LCDInit();
  LCDContrast (0x60);
  LCDClear();
  LCDUpdate();
}

WebServer server(80);
// Init WebServer for remote management
void handleRoot(){  // Page d'accueil La page HTML est mise dans le String page
  String page = "<!DOCTYPE html>";  // Début page HTML
    page += "<head>";
    page += "    <title>Serveur ESP32</title>";
    page += "    <meta http-equiv='refresh' content='60' name='viewport' content='width=device-width, initial-scale=1' charset='UTF-8'/>";
    page += "</head>";
    page += "<body lang='fr'>";
    page += "    <h1>MIAGE - OCSI : Server Web Embarqué</h1>";
    page += "    <p>Device Information</p>";
    page += "    <p>SENSOR_GPS :" + String(SENSOR_GPS) + "</p>";
    page += "    <p>SENSOR_LCD :" + String(SENSOR_LCD) + "</p>";
    page += "    <p>SENSOR_MAG :" + String(SENSOR_MAG) + "</p>";
    page += "    <p>SENSOR_WEATHER :" + String(SENSOR_WEATHER) + "</p>";
    page += "</body>";
    page += "</html>";  // Fin page HTML

    server.send(200, "text/html", page);  // Envoie de la page HTML
}

void handleNotFound(){  // Page Not found
  server.send(404, "text/plain","404: Page Introuvable");
}

void initWebServer() {
  server.on("/", handleRoot);  // Chargement de la page d'accueil
  server.onNotFound(handleNotFound);  // Chargement de la page "Not found"
  server.begin();  // Initialisation du serveur web
}


/////////////////////////////////////////////////
// Board Setup function (launched at startup) //
///////////////////////////////////////////////
void setup() {
  // Initialize Board
  initBoard();
  // Init Devices based on Configuration
if (SENSOR_WEATHER)
  initWeather();
if (SENSOR_GPS)
  initGps();
if (SENSOR_LCD)
  initLCD();
if (SENSOR_LIGHT)
  initLightSensor();

// Always init the WebServer
initWebServer();
}



////////////////////////
// Main Program Loop //
//////////////////////
void loop() {
  String thingName = getUniqueDeviceName(); //unique name for this Thing so many work on one ThingWorx server
  int i = 12;
  while (WiFi.status() == WL_CONNECTED) { //confirm WiFi is connected before looping as long as WiFi is connected
  // Managing WebClient :
  server.handleClient();
    // Acquire Values from the sensors depending on configuration
     if (SENSOR_MAG) sendMagData(thingName);
     if (SENSOR_LCD) displayTest();
     if (SENSOR_LIGHT) sendLightData(thingName);
     
     if (SENSOR_WEATHER)
    if (ccs.available()) {
      if (!ccs.readData()) {
        float co2 = ccs.geteCO2();
        float tvoc = ccs.getTVOC();
        float temp = bme.readTemperature();
        float hum = bme.readHumidity();
        float pres = bme.readPressure();
        // publish properties :
        thingworx.httpPutPropertry(thingName, String(CO2_PROPERTY), String(co2));
        thingworx.httpPutPropertry(thingName, String(TVOC_PROPERTY), String(tvoc));
        thingworx.httpPutPropertry(thingName, String(HUMIDITY_PROPERTY), String(hum));
        thingworx.httpPutPropertry(thingName, String(TEMPERATURE_PROPERTY), String(temp));
        thingworx.httpPutPropertry(thingName, String(PRESSURE_PROPERTY), String(pres));
        // Get Location from the GPS :
        while (gpsSerial.available() > 0)
          if (gps.encode(gpsSerial.read()))
            gps.displayInfo();
        thingworx.httpPutPropertry(thingName, "Location", "{\"latitude\":\"" + String(gps.location.lat()) + "\",\"longitude\":\"" + gps.location.lng() + "\",\"units\":\"WGS84\"}");
      }
    }
    i++;
    // Sends every INTERVAL mseconds
    delay(INTERVAL);
  }// end WiFi connected while loop
  Serial.printf("****Wifi connection dropped****\n");
  WiFi.disconnect(true);
  
}
