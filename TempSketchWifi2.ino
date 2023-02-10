#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
 
#define DS18B20 D2          //DS18B20 is connected to GPIO Pin 2

String apiKey = "DEUQ1ZQ65FV2ZPUE";     //  Enter your Write API key from ThingSpeak
const char* ssid =  "Breck-EWN-2.4";     // Enter your WiFi Network's SSID
const char* pass =  "Skiing4All"; // Enter your WiFi Network's Password
const char* server = "api.thingspeak.com";
float temp;
 
OneWire ourWire(DS18B20);
DallasTemperature sensor(&ourWire);
WiFiClient client;
 
void setup() 
{
       Serial.begin(115200);
       delay(1000);
       sensor.begin();
 
       Serial.print("Connecting to: ");
       Serial.println(ssid);
 
 
       WiFi.begin(ssid, pass);
 
      while (WiFi.status() != WL_CONNECTED) 
     {
            delay(100);
            Serial.print("*");
     }
      Serial.println("");
      Serial.println("***WiFi connected***");
 
}
 
void loop() 
{
  
      sensor.requestTemperatures();
      temp = sensor.getTempFByIndex(0);
 
      if (client.connect(server,80))   //   "184.106.153.149" or api.thingspeak.com
      {  
       String sendData = apiKey+"&field1="+String(temp)+"\r\n\r\n"; 
       
       //Serial.println(sendData);

       client.print("POST /update HTTP/1.1\n");
       client.print("Host: api.thingspeak.com\n");
       client.print("Connection: close\n");
       client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
       client.print("Content-Type: application/x-www-form-urlencoded\n");
       client.print("Content-Length: ");
       client.print(sendData.length());
       client.print("\n\n");
       client.print(sendData);

       Serial.print("Temperature: ");
       Serial.print(temp);
       Serial.println("deg F. Connecting to Thingspeak..");
      }
      
      client.stop();

      Serial.println("Sending....");
  
 delay(300000);
}
