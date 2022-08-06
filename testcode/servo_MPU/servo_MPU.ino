//for servomoter
#include <ESP_servo.h>
#include <stdio.h>
#include <string>

#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;


ESP_servo servo1;

void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("started");
    
    #ifdef _ESP32_HAL_I2C_H_ // For ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
    mySensor.setWire(&Wire);
    #endif
    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();
    
    //for servomoter
    servo1.init(19,1);
    servo1.write(90);
    

}//setup関数閉じ


void loop() {
  uint8_t sensorId;
  if (mySensor.readId(&sensorId) == 0) {
    Serial.println("sensorId: " + String(sensorId));
  } else {
    Serial.println("Cannot read sensorId");
  }

  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
  } else {
    Serial.println("Cannod read accel values");
  }

  if(aZ < -0.95){
    servo1.write(175);//引数は角度(°)
    delay(1000);
  }
  else if(aZ > 0.85){
    servo1.write(5);
    delay(1000);
  }


}//loop関数の閉じ
