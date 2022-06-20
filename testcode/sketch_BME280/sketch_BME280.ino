#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

float temp;
float pressure;
float humid;

void setup() {
  Serial.begin(115200);
  bool status;
  status = bme.begin(0x76);  
    delay(1000);
 
}
void loop() { 
  temp=bme.readTemperature();
  pressure=bme.readPressure() / 100.0F;
  humid=bme.readHumidity();
  Serial.print("温度 : ");
  Serial.print(temp);
  Serial.println(" °C");
   
  Serial.print("気圧 : ");
  Serial.print(pressure);
  Serial.println(" hPa");
  
  Serial.print("湿度 : ");
  Serial.print(humid);
  Serial.println(" %");
  Serial.println();
  delay(1000);
}
