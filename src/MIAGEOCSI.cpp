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
#include <lcd3310_GPIO.h>
#include <MODMAG.h>
#include <WebServer.h>
#include <ltr501.h>
#include <Wire.h>
#include <BMP085.h>
#include <MPU6050.h>
#include <MODSMB380.h>
#include <OneWireTemp.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>

#define OUTPUT_READABLE_ACCELGYRO

	#define RELAY1_MAKE_OUT()     pinMode (32, OUTPUT)
	#define RELAY1_HIGH()         digitalWrite (32, 1)
	#define RELAY1_LOW()          digitalWrite (32, 0)

	#define RELAY2_MAKE_OUT()     pinMode (33, OUTPUT)
	#define RELAY2_HIGH()         digitalWrite (33, 1)
	#define RELAY2_LOW()          digitalWrite (33, 0)

  #define BUTTON KEY_BUILTIN
  #define BUTTON_MAKE_IN()     pinMode (BUTTON, INPUT)
	//#define RELAY2_HIGH()         digitalWrite (33, 1)
	//#define RELAY2_LOW()          digitalWrite (33, 0)

//////////////////////////
// WiFi Configurations //
//  TO CHANGE         //
///////////////////////
 char WiFiSSID[] = "SSIDDEFAULTVALUESSIDDEFAULTVALUE"; // WiFi access point SSID
 char WiFiPSK[] = "PWDDEFAULTVALUEPWDDEFAULTVALUEPWD"; // WiFi password - empty string for open access points
String SSIDToStore="";
String PWDToStore="";
String HttpHeader = String(16);


//////////////////////////////////////////////
// ThingWorx server definitions            //
//  TO CHANGE                             //
///////////////////////////////////////////
String TWPlatformBaseURL = "http://miage.mecanaute.com:22222";
String APP_KEY = "9f005f65-c537-4607-9f1f-b77d6ebed9b9";
const int INTERVAL = 10; //refresh interval
const char THING_PREFIX[] = "MIAGE.";
const char CO2_PROPERTY[] = "WEATHER_CO2";                 //Thing property name
const char TVOC_PROPERTY[] = "WEATHER_AirQuality";         //Thing property name
const char HUMIDITY_PROPERTY[] = "WEATHER_Humidity";       //Thing property name
const char TEMPERATURE_PROPERTY[] = "WEATHER_Temperature"; //Thing property name
const char PRESSURE_PROPERTY[] = "WEATHER_Pressure";       //Thing property name
ThingworxRest thingworx(TWPlatformBaseURL,APP_KEY);
//////////////////////////////////////////////
// Program execution settings              //
//  TO CHANGE                             //
///////////////////////////////////////////
bool debug=true;
bool info=true;
bool SENSOR_WEATHER=false; // Weather Sensors
bool SENSOR_GPS=false;
bool SENSOR_LCD=true;
bool SENSOR_MAG=false;
bool SENSOR_LIGHT=false;
bool SENSOR_BARO=false;
bool SENSOR_ACCEL=false;
bool SENSOR_SMB380=false;
bool SENSOR_TEMP=false;

// Definitions for SMB380
//#define CLOCK 14    // SPI Clock
//#define CS    17     // Chip Select
//#define MOSI  2    // Master OUT - Slave IN line
//#define MISO  15    // Master IN  - Slave OUT line


//////////////////////////////////////////////////////////
// Pin Definitions - board specific for Adafruit board //
////////////////////////////////////////////////////////
Adafruit_CCS811 ccs;
Adafruit_BME280 bme; // I2C
//const int RED_LED = 0; // Thing's onboard, red LED -
//const int BLUE_LED = 2; // Thing's onboard, blue LED
//const int ANALOG_PIN = 0; // The only analog pin on the Thing
//const int OFF = HIGH;
//const int ON = LOW;
// this will set as the Accept header for all the HTTP requests to the ThingWorx server
// valid values are: application/json, text/xml, text/csv, text/html (default)

///////////////////
//MOD GPS SETUP : all moved to the utility Class
/////////////////
unsigned long start;
HardwareSerial gpsSerial(Serial1);
#define BAUDRATE 9600 // this is the default baudrate of the GPS module
//GPS
TinyGPSPlus gps;
// Barometer 
BMP085 bmp085 = BMP085(0x77);
// Mod Magnetometer
MODMAG modmag;
// AccelGyro
MPU6050 accelgyro;
// SMB380

MODSMB380 smb380(14, 40, 2, 15);
 
// EEPROM Utilities

/*
Writes the value to the EEPROM :
BIT 1 = String size
then write bit per bit
*/
void writeStringToEEPROM(String value,int address)
{
  // write to the eeprom the size of the String :
  EEPROM.write(address,value.length()); 
  // Write char by char :
  for (int i=1;i<=value.length();i++)
  {
    int writeAdd = address+i;
    EEPROM.write(writeAdd,value[i-1]);

  }
 EEPROM.commit();
}


/*
Read String from EEProm Adress :
BIT 1 : String size.
Reads the number of bytes (max 255)
*/
String readStringFromEEPROM(int address)
{
  String result ="";
  // Read the String size :
  int size = EEPROM.read(address);
  if (size > 255) size = 255;
  for (int i=1;i<=size; i++)
  {
    // Read the value : 
    int charencoded = EEPROM.read(address + i);
    char decoded = charencoded;
    result += decoded;
  }
return result;
}




void sendMagData(String thingname){
  //unsigned char aflag;
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

// Send the Baro Data
void sendBaroData(String thingname) {
  bmp085.readMeasurement();
  int temp =bmp085.getTemperature();
  if (debug) Serial.print("TEMPERATURE: ");
  if (debug) Serial.print(temp , 2);
  thingworx.httpPutPropertry(thingname,"BARO_TEMP",String(temp));

  int pressure =bmp085.getPressure();
  if (debug) Serial.print("PRESSURE: ");
  if (debug) Serial.print(pressure, 3);
  if (debug) Serial.println("hPa");
  if (debug) Serial.println();
  thingworx.httpPutPropertry(thingname,"BARO_PRESSURE",String(pressure));
}

// Send the Light Data
void sendLightData(String thingname) {
  int lux = getLighSensorMeasure();
  int distance = getLighSensorDistance();
  
  thingworx.httpPutPropertry(thingname,"LIGHT_LUX",String(lux));
  thingworx.httpPutPropertry(thingname,"LIGHT_DIST",String(distance));
  
}
// LCD Display Management
void lcd(String v, int row, int invert) {
   char  buffer[20];
   v.toCharArray(buffer,20);
    LCDStr(row, (unsigned char *) buffer, invert);
}

// Send smb380 Data 
void sendSMB380Data(String thingname){
    smb380.updateData();
    thingworx.httpPutPropertry(thingname,"SMB380_AX",String(smb380.getAccX()));
    thingworx.httpPutPropertry(thingname,"SMB380_AY",String(smb380.getAccY()));
    thingworx.httpPutPropertry(thingname,"SMB380_AZ",String(smb380.getAccZ())); 

}
// Send Temperature Data 
void sendTempData(String thingname){
    thingworx.httpPutPropertry(thingname,"TEMP_TEMP",String(readTemperature())); 
}

// Send the AccelGyro Data
void sendAccelData(String thingname) {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

        // display tab-separated accel/gyro x/y/z values
        if (debug) {
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
        }
  
  thingworx.httpPutPropertry(thingname,"ACCEL_AX",String(ax));
  thingworx.httpPutPropertry(thingname,"ACCEL_AY",String(ay));
  thingworx.httpPutPropertry(thingname,"ACCEL_AZ",String(az));
  thingworx.httpPutPropertry(thingname,"ACCEL_GX",String(gx));
  thingworx.httpPutPropertry(thingname,"ACCEL_GY",String(gy));
  thingworx.httpPutPropertry(thingname,"ACCEL_GZ",String(gz));
}

void sendGPSData(String thingName) {
          while (gpsSerial.available() > 0)
          if (gps.encode(gpsSerial.read()))
            gps.displayInfo();
        thingworx.httpPutPropertry(thingName, "Location", "{\"latitude\":\"" + String(gps.location.lat()) + "\",\"longitude\":\"" + gps.location.lng() + "\",\"units\":\"WGS84\"}");

}

void sendWeatherData(String thingName) {
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
      }
    }
}






/////////////////////
// WiFi connection. Checks if connection has been made once per second until timeout is reached
// returns TRUE if successful or FALSE if timed out
/////////////////////
boolean connectToWiFi(int timeout, String epromSSID, String epromPWD) {
  // Fetching values from EEPROM and initializing the char arrays 
  char epromSSIDc[epromSSID.length()+1];
  char epromPWDc[epromPWD.length()+1];
  if (epromSSID != "") {
    // Convert the values : 
    epromSSID.toCharArray(epromSSIDc,epromSSID.length()+1);
    epromPWD.toCharArray(epromPWDc,epromPWD.length()+1);
  }
  
  Serial.println("Connecting to: (" + String(epromSSIDc) +") With (" + String(epromPWDc)+")");

  WiFi.begin(epromSSIDc, epromPWDc);
  // loop while WiFi is not connected waiting one second between checks
  uint8_t tries = 0; // counter for how many times we have checked
  while ((WiFi.status() != WL_CONNECTED) && (tries < timeout) ) { // stop checking if connection has been made OR we have timed out
    tries++;
    Serial.printf(".");// print . for progress bar
    //Serial.println(WiFi.status());
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
  //Serial.println("DeviceID>" + mac);
  return String(THING_PREFIX + mac.substring(6));
}

// Create the AP for Wifi connection
boolean createAccessPoint(int timeout){

   char  buffer[20];
   String apname = getUniqueDeviceName();
    apname.toCharArray(buffer,20);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(buffer,"MIAGEOCSI");
    Serial.print("[+] AP Created with IP Gateway ");
    Serial.println(WiFi.softAPIP());
 return WiFi.isConnected();
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


void displayValues() {
//if (debug) Serial.println("LCD Display Test");
    lcd("MIAGE  -  OCSI",0,0);
    lcd(" " + getUniqueDeviceName(),1,0);
    LCDRectangle(0,16,83,45);
    // Fetch Values from Thingworx property : 
    String l1 = thingworx.httpGetStringPropertry(getUniqueDeviceName(),"LCD_Line1");
    String l2 = thingworx.httpGetStringPropertry(getUniqueDeviceName(),"LCD_Line2");
    String l3 = thingworx.httpGetStringPropertry(getUniqueDeviceName(),"LCD_Line3");
    lcd(l1,2,0);
    lcd(l2,3,0);
    lcd(l3,4,0);
    //LCDTriangle(10,10,80,45,0,47);
    //LCDCircle(50,30,10);
    //LCDRectangle(20,20,40,40);
    LCDRectangle(0,16,83,45);
    LCDUpdate();    
  }

void initLCD() {
  if (debug) Serial.println("Init LCD");
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  LCDInit();
  LCDContrast (0x60);
  if (debug) Serial.println("Clear LCD");
  LCDClear();
  LCDUpdate();
  if (debug) Serial.println("Init  LCD Done");

}

void initBaro(){
  bmp085.loadConstants();
  bmp085.setOSS(2);
}


void initAccel(){
      accelgyro.initialize();
}

//WebServer server(80);
AsyncWebServer server(80);


void ReloadConfiguration() {
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
  SENSOR_BARO =  thingworx.httpGetBoolPropertry(thingname,"SENSOR_BARO"); 
  SENSOR_ACCEL =  thingworx.httpGetBoolPropertry(thingname,"SENSOR_ACCEL"); 
  SENSOR_SMB380 = thingworx.httpGetBoolPropertry(thingname,"SENSOR_SMB380"); 
  SENSOR_TEMP = thingworx.httpGetBoolPropertry(thingname,"SENSOR_TEMP"); 
if (SENSOR_WEATHER)
  initWeather();
if (SENSOR_GPS)
  initGps();
if (SENSOR_LIGHT)
  initLightSensor();
if (SENSOR_BARO)
  initBaro();
if (SENSOR_ACCEL)
  initAccel();
}
// Init WebServer for remote management
void ReloadConfigurationFromWeb(AsyncWebServerRequest *request) {
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
  SENSOR_BARO =  thingworx.httpGetBoolPropertry(thingname,"SENSOR_BARO"); 
  SENSOR_ACCEL =  thingworx.httpGetBoolPropertry(thingname,"SENSOR_ACCEL"); 
  SENSOR_SMB380 = thingworx.httpGetBoolPropertry(thingname,"SENSOR_SMB380"); 
  SENSOR_TEMP = thingworx.httpGetBoolPropertry(thingname,"SENSOR_TEMP"); 
if (SENSOR_WEATHER)
  initWeather();
if (SENSOR_GPS)
  initGps();
if (SENSOR_LIGHT)
  initLightSensor();
if (SENSOR_BARO)
  initBaro();
if (SENSOR_ACCEL)
  initAccel();

 String page = "<!DOCTYPE html>";  // Début page HTML
    page += "<head>";
    page += "    <title>MIAGE OCSI - Device Reconfiguration</title>";
    page += "    <meta http-equiv='refresh' content='60' name='viewport' content='width=device-width, initial-scale=1' charset='UTF-8'/>";
    page += "</head>";
    page += "<body lang='fr'>";
    page += "    <h1>MIAGE - OCSI : Device Reconfiguration complete</h1>";
    page += "    <h2>Thing Name :" + getUniqueDeviceName() +"</h2>";
    page += "    <a href='/'>Continue</a>";
    page += "</body>";
    page += "</html>";  // Fin page HTML
    char  pagec[2500];
    page.toCharArray(pagec,2500);
    request->send_P(200, "text/html", pagec); 
    
  //  request->send_P(page);
}

void manageBase(String thingname) {
   boolean relay1 = thingworx.httpGetBoolPropertry(thingname,"BASE_RELAY1"); 
   boolean relay2 = thingworx.httpGetBoolPropertry(thingname,"BASE_RELAY2"); 
   if (relay1) RELAY1_HIGH() ; 
    else RELAY1_LOW();
   if (relay2) RELAY2_HIGH() ; 
    else RELAY2_LOW();
  thingworx.httpPutPropertry(thingname,"BASE_BUTTON",String(digitalRead(BUTTON)));
}


 // Init Debug Output to tty
void initBoard() {
  EEPROM.begin(99);
  String epromSSID, epromPwd;
  Serial.begin(9600);
  Serial.println("Fetched Network configuration from EEPROM : ");
  epromSSID = readStringFromEEPROM(0);
  epromPwd = readStringFromEEPROM(50);
  Serial.println(epromSSID);
  Serial.println(epromPwd);
  RELAY1_MAKE_OUT();
  RELAY2_MAKE_OUT();
  BUTTON_MAKE_IN();
  initLCD();
  lcd("Booting....",0,1);
  Serial.setDebugOutput(true);
  delay(2000);
  Serial.flush();
  Serial.println("Starting Firmware...");
  Serial.println("Done!");
// Connect  Wifi  
  boolean connected = connectToWiFi(10,epromSSID,epromPwd);
  if (!connected) {
    createAccessPoint(10);
  }
  lcd(getUniqueDeviceName(),0,1);
  lcd(WiFi.localIP().toString(),1,0);
  //Serial.println(RST);
  Serial.flush();
  // Fetch configuration from the Thingworx Server
  ReloadConfiguration();
}

void handleRoot(AsyncWebServerRequest *request){  // Page d'accueil La page HTML est mise dans le String page
  String page = "<!DOCTYPE html>";  // Début page HTML
    page += "<head>";
    page += "    <title>MIAGE OCSI - Device Information</title>";
    page += "    <meta http-equiv='refresh' content='60' name='viewport' content='width=device-width, initial-scale=1' charset='UTF-8'/>";
    page += "</head>";
    page += "<body lang='fr'>";
    page += "    <h1>MIAGE - OCSI : Server Web Embarqué</h1>";
    page += "    <h2>Thing Name :" + getUniqueDeviceName() +"</h2>";
    page += "    <p>Device Information</p>";
    page += "    <p>Log DEBUG       :" + String(debug) + "</p>";
    page += "    <p>Log INFO        :" + String(info) + "</p>";  
    page += "    <p>SENSOR_GPS      :" + String(SENSOR_GPS) + "</p>";
    page += "    <p>SENSOR_LCD      :" + String(SENSOR_LCD) + "</p>";
    page += "    <p>SENSOR_MAG      :" + String(SENSOR_MAG) + "</p>";
    page += "    <p>SENSOR_WEATHER  :" + String(SENSOR_WEATHER) + "</p>";
    page += "    <p>SENSOR_LIGHT    :" + String(SENSOR_LIGHT) + "</p>";
    page += "    <p>SENSOR_BARO     :" + String(SENSOR_BARO) + "</p>";
    page += "    <p>SENSOR_SMB380   :" + String(SENSOR_SMB380) + "</p>";
    page += "    <p>SENSOR_ACCEL    :" + String(SENSOR_ACCEL) + "</p>";
    page += "    <p>SENSOR_TEMP     :" + String(SENSOR_TEMP) + "</p>";
    page += "    <a href='/reconf'>Reload Confiugration</a>";
    page += "    <a href='/reset'>Reset Board</a>";
    page += "<h1> WIFI Reconfiguration </H1>";
    page += "<form action='/get'>";
    page += "SSID : <input type='text'name='ssid'> </input>";
    page += "PWD  : <input type='text'name='pwd'> </input>";
    page += "<input type=submit value=submit></form>";
    page += "</body>";
    page += "</html>";  // Fin page HTML

    char  pagec[2500] ;
     page.toCharArray(pagec,2500);
    request->send_P(200, "text/html", pagec);  // Envoie de la page HTML
}

void handleNotFound(AsyncWebServerRequest *request){  // Page Not found
  request->send_P(404, "text/plain","404: Page Introuvable");
}
void RebootDevice(AsyncWebServerRequest *request) {
  // Reboot board :

}

void initWebServer() {
  server.on("/",HTTP_GET, handleRoot);  // Chargement de la page d'accueil
  server.on("/reconf",HTTP_GET, ReloadConfigurationFromWeb);  // Chargement de la page d'accueil
  server.on("/reboot",HTTP_GET, RebootDevice);
  server.onNotFound(handleNotFound);  // Chargement de la page "Not found"
   server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam("ssid")) {
      SSIDToStore = request->getParam("ssid")->value();
      //SSID = PARAM_INPUT_1;
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    if (request->hasParam("pwd")) {
      PWDToStore = request->getParam("pwd")->value();
      //inputParam = PARAM_INPUT_2;
    }
    Serial.println("Config Stored : ");
    Serial.println(SSIDToStore);
    Serial.println(PWDToStore);
    writeStringToEEPROM(SSIDToStore,0);
    writeStringToEEPROM(PWDToStore,50);
    // Store the new NetConfig
    
  request->send_P(200,"text/html","Configuration Updated");

   });
  // Initialisation du serveur web
  server.begin();
}


/////////////////////////////////////////////////
// Board Setup function (launched at startup) //
///////////////////////////////////////////////
void setup() {
    HttpHeader="";
  // Initialize Board
  initBoard();
// Always init the WebServer
  initWebServer();
}
// Reads the http input : 



////////////////////////
// Main Program Loop //
//////////////////////
void loop() {
  
  String thingName = getUniqueDeviceName(); //unique name for this Thing so many work on one ThingWorx server
  int i = 12;
  while (WiFi.status() == WL_CONNECTED) { //confirm WiFi is connected before looping as long as WiFi is connected
  // Managing WebClient :
   // server.handleClient();
    manageBase(thingName);
    // Acquire Values from the sensors depending on configuration
     if (SENSOR_MAG) sendMagData(thingName);
     if (SENSOR_LCD) displayValues();
     if (SENSOR_LIGHT) sendLightData(thingName);
     if (SENSOR_BARO) sendBaroData(thingName);
     if (SENSOR_ACCEL) sendAccelData(thingName);
     if (SENSOR_SMB380) sendSMB380Data(thingName);
     if (SENSOR_TEMP) sendTempData(thingName);
     if (SENSOR_GPS) sendGPSData(thingName);
     if (SENSOR_WEATHER) sendWeatherData(thingName);
    i++;
    // Sends every INTERVAL mseconds
    delay(INTERVAL);
  }// end WiFi connected while loop

  if (WiFi.status() != WL_CONNECTED) {
  // booting up in AP Mode : no Wifi Availlable.
  lcd("   AP MODE    ",0,1);
  lcd(getUniqueDeviceName(),1,0);
  lcd("MIAGEOCSI",2,0);
  lcd(WiFi.softAPIP().toString(),3,0);
  //    server.handleClient();
  }
  WiFi.disconnect(true);
  
}
