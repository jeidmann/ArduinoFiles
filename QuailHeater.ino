#include <OneWire.h> 
#include <DallasTemperature.h>

#define DHT11_PIN 7
#define ONE_WIRE_BUS 12 

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

int pinOut = 10;

void setup(){
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  sensors.begin(); 
}

void loop()
{
  sensors.requestTemperatures();
  Serial.print("Temperature = ");
  Serial.println(sensors.getTempCByIndex(0));
  if (sensors.getTempCByIndex(0)<= 15){
    digitalWrite(pinOut, HIGH);
    delay(480000);
  }
  else {
    digitalWrite(pinOut, LOW);
    delay(480000);
  }
  delay(2000);            
}
