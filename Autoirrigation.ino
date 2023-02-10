/*
  Simple Irrigation Control, SOCR581A3 2019
  Use an ESP32 Huzzah32 to read two soil moisture sensors and a water level sensor 
  Sends data to Adafruit.io dashboard via WiFi every 10s. 
  Allows control of soil moisture threshold (how dry soil must be before irrigate)
  Allow control of irrigation duration.
  Allow manual override - trigger manual irrigation
  Tested on Huzzah32 board https://www.adafruit.com/product/3405
  Requires install of: Adafruit IO library, Adafruit MQTT library, and Arduinohttpclient library
  Also install meanfilter library
  Requires two capacitance based sensors: V1.2 DIYMORE
  Requires use of non contact water sensor for water reservoir,XKC-Y25-PNP
  
  J Ham, CSU, 21Nov2019

  Project similar to this tutorial (get some pointers here)
  https://learn.adafruit.com/adafruit-io-basics-temperature-and-humidity/overview    
*/

#include "Wire.h"
#include "AdafruitIO_WiFi.h"    // Adafruits WiFi Library (part of Adafruit IO Arduino library)
#include "MeanFilterLib.h"

// Your WiFi information, on campus
#define WIFI_SSID "w212lab"        //  your network SSID (name) 
#define WIFI_PASS "testarduino"     // your network password

// Your WiFi information, off campus
//#define WIFI_SSID "HOME-D289-2.4"   // your network SSID (name) 
//#define WIFI_PASS "393T4UC43YA44CU3"   // your network password

// Adafruit IO information for your account
#define IO_USERNAME    "micrometeo"
#define IO_KEY         "c515cee8d740a3d03dbfacd69c66daa3df2e811f"

// global Objects, setting up wifi
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

//define the feeds to Adafruit IO
// variable names in io.feed must match adafruit io feeds, t-sht and rh-sht.
AdafruitIO_Feed *soilFeed_1 = io.feed("soil-1");   
AdafruitIO_Feed *soilFeed_2 = io.feed("soil-2");
AdafruitIO_Feed *waterFeed = io.feed("water");
AdafruitIO_Feed *autoirr = io.feed("auto");
AdafruitIO_Feed *override = io.feed("override");
AdafruitIO_Feed *irrtimer = io.feed("irr-timer");
AdafruitIO_Feed *irrsetpoint = io.feed("irr-setpoint");

// Pin Assignments
#define LED_AUTO_PIN 27
#define LED_OVERRIDE_PIN 12
#define SOIL1 A2      // analog channel for soil sensor 1, A2 on huzzah
#define SOIL2 A3      // analog channel for soil sensor 2, A3 on huzzah
#define WATER_PIN 15  // digital input pin for water level sensor in tank 
#define PUMP_PIN 27   // digital output pin for turning on water pump

MeanFilter<float> meanSoil_1(6);      // mean calculations for soil sensors
MeanFilter<float> meanSoil_2(6);      // mean calculations for soil sensors

const float wet = 1.3;  // 100%    voltage output from soil sensors in water saturated soil
const float dry = 2.9;  // 0*      voltage output from soil sensors in air dry soil

unsigned long previousMillis = 0;        // will store last time updated
const long interval = 10000;             // sampling interval, 10s
bool autoirr_Flag;   //flag - high when auto irrigate is active, low when on manual
bool override_Flag;   //flag - high - trigger manual irrigation
bool water_Flag = false;  // indicates when getting low on water in supply tank/bottle
int irr_time;         // duration of irrigation in seconds
int irr_setpoint;

void setup()
{
  Serial.begin(9600);
  delay(2000);
  
  pinMode(LED_AUTO_PIN, OUTPUT);
  pinMode(LED_OVERRIDE_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(WATER_PIN, INPUT);
  
  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  autoirr->onMessage(handleMessage1);
  override->onMessage(handleMessage2);
  irrsetpoint->onMessage(handleMessage3);
  irrtimer->onMessage(handleMessage4);

  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

   // we are connected
  Serial.println();
  Serial.println(io.statusText());

  //get initial values from controls on dashboard
  autoirr->get();
  irrtimer->get();
  irrsetpoint->get();
}

void loop()
{
  // io.run(); is required for all sketches.
  io.run();
  
  unsigned long currentMillis = millis();   // read mills timer
  if (currentMillis - previousMillis >= interval) {  //control sampling interval
    previousMillis = currentMillis;

  //Read soil moisture sensors
  float volt_soil_1 = meanSoil_1.AddValue(ReadVoltage(SOIL1)); // read voltages and average
  float volt_soil_2 = meanSoil_2.AddValue(ReadVoltage(SOIL2)); // read voltages and average
  Serial.print("volts <- ");
  Serial.print(volt_soil_1);Serial.print(", ");
  Serial.println(volt_soil_2);
  
  // map voltage to percent
  float percent_soil_1=mapfloat(volt_soil_1,dry,wet,0.0,100.0);
  float percent_soil_2=mapfloat(volt_soil_2,dry,wet,0.0,100.0);
  
  Serial.print("percent <- ");
  Serial.print(percent_soil_1);Serial.print(", ");
  Serial.println(percent_soil_2);
  Serial.println();

  //Read water level sensor
  if(digitalRead(WATER_PIN) == LOW)
    water_Flag = true;
   else
    water_Flag = false;
  Serial.print("water pin <- ");
  Serial.println(digitalRead(WATER_PIN));  
  Serial.print("water flg <- ");
  Serial.println(water_Flag);
   
   // Send data to Adafruit IO
  soilFeed_1->save(percent_soil_1);
  soilFeed_2->save(percent_soil_2);
  waterFeed->save(water_Flag);
  }
}  // end main loop

// Auxillary Functions

//function to map floating point numbers (used for soil moisture)
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//function to correct ADC, https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function
double ReadVoltage(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
} 

void handleMessage1(AdafruitIO_Data *data) {

  Serial.print("received auto <- ");

  if(data->toPinLevel() == HIGH)
    Serial.println("HIGH");
  else
    Serial.println("LOW");


  digitalWrite(LED_AUTO_PIN, data->toPinLevel());
}

void handleMessage2(AdafruitIO_Data *data) {

  Serial.print("received override <- ");

  if(data->toPinLevel() == HIGH)
    Serial.println("HIGH");
  else
    Serial.println("LOW");


  digitalWrite(LED_OVERRIDE_PIN, data->toPinLevel());
  delay(1000);
}

void handleMessage3(AdafruitIO_Data *data) {
 
  // convert the data to integer
  irr_time = data->toInt();
 
  Serial.print("received setpoint <- "); 
  Serial.println(irr_time);
   
}

void handleMessage4(AdafruitIO_Data *data) {
 
  // convert the data to integer
  int irr_setpoint = data->toInt();
 
  Serial.print("received timer <- "); 
  Serial.println(irr_setpoint);
   
}
