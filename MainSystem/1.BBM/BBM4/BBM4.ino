//for PI
#include <math.h>

// for MPU6050
#include <MPU9250_asukiaaa.h>
#ifdef _ESP32_HAL_I2C_H_
#define SDA_MPU 21 // I2C通信
#define SCL_MPU 22
#endif
MPU9250_asukiaaa mySensor;

// for BME280
#include <Wire.h> // I2C通信
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

// for GPS
#include <TinyGPS++.h>
TinyGPSPlus gps;

// for GY-271
#include <DFRobot_QMC5883.h>
DFRobot_QMC5883 compass;
int CalibrationCounter = 1; // キャリブレーションで取得したデータの個数
double declinationAngle; // 補正用
double heading; // 弧度法
double headingDegrees; // 度数法
double Sum_headingDegrees; //連続15個のheadingDegreesの和
double Angle_gy271; // 連続15個の平均値(度数法)

// for SD Card
#include <SPI.h> // SDカードはSPI通信
#include <SD.h>
File CanSatLogData;
File SensorData;
const int sck = 13, miso = 15, mosi = 14, ss = 27;

// for HY-SRF05
#define echoPin 35 // Echo Pin
#define trigPin 4 // Trigger Pin
double Duration = 0; // 受信した間隔

// センサー値を格納するための変数
float Temperature, Pressure, Humid; // for BME280
double accelX, accelY, accelZ, gyroX, gyroY, gyroZ, accelSqrt; // for MPU6050
double gps_latitude, gps_longitude, gps_velocity; // for GPS
int gps_time; // for GPS
double ultra_distance = 0; // for HY-SRF05

// for motor
// 前進
void forward()
{
  ledcWrite(0, 0); // channel, duty
  ledcWrite(1, 200);
  ledcWrite(2, 0);
  ledcWrite(3, 200);
}
// 停止
void stoppage()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}
// 回転
void rotating()
{
  ledcWrite(0, 0);
  ledcWrite(1, 200);
  ledcWrite(2, 200);
  ledcWrite(3, 0);
}
// 反回転
void reverse_rotating()
{
  ledcWrite(0, 200);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 200);
}

// for servomoter
#include <ESP_servo.h>
#include <stdio.h>
#include <string>
ESP_servo servo1;

// for parachute
int cutparac = 23;          // 切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 3;    // 切り離し時の9V電圧を流す時間，単位はsecond
int phase_state = 0;
char key = '0';

// for ガイガーカウンタ
char data;

void setup()
{
  // for GPS
  Serial1.begin(9600, SERIAL_8N1, 5, 18); // ESP32-GPS間のシリアル通信開始
  
  // SD Card initialization
  SPI.begin(sck, miso, mosi, ss);
  SD.begin(ss, SPI);
  CanSatLogData = SD.open("/CanSatLogData.txt", FILE_APPEND);
  SensorData = SD.open("/SensorData.csv", FILE_APPEND);
  CanSatLogData.println("START_RECORD");
  CanSatLogData.flush();
  SensorData.println("gps_time,gps_latitude,gps_longitude,gps_velocity,Temperature,Pressure,Humid,accelX,accelY,accelZ,Angle_gy271,ultra_distance");
  SensorData.flush();

  // for Serial communication
  Serial2.begin(115200, SERIAL_8N1,16,17); // ESP32-twelite間のシリアル通信開始
  Serial2.println("TESTING:SERIAL COMMUNICATION");

  // for ガイガーカウンタ
  Serial.begin(38400, SERIAL_8N1,34,2); // ESP32-ガイガーカウンタ間のシリアル通信開始
  
  // for MPU6050
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  
  // for BME280
  bool status;
  status = bme.begin(0x76);

  // for ESP32
  #ifdef _ESP32_HAL_I2C_H_
  Wire.begin(SDA_MPU, SCL_MPU);
  mySensor.setWire(&Wire);
  #endif

  // for motor
  ledcSetup(0, 490, 8);
  ledcSetup(1, 490, 8);
  ledcSetup(2, 490, 8);
  ledcSetup(3, 490, 8);

  ledcAttachPin(32, 0);
  ledcAttachPin(33, 1);
  ledcAttachPin(26, 2);
  ledcAttachPin(25, 3);

  // for GY-271
  // GY-271の初期化
  while (!compass.begin())
  {
    Serial2.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }
  if (compass.isHMC())
  {
    Serial2.println("Initialize HMC5883");
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_15HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
  }
  else if (compass.isQMC())
  {
    Serial2.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
  // GY-271の初期化終了
  
  // GY-271のキャリブレーション
  delay(100);
  Serial2.println("calibration rotating!");
  while (CalibrationCounter < 551)
  {
    Vector norm = compass.readNormalize();
    rotating();
    if (CalibrationCounter == 550)
    {
      stoppage();
      Serial2.println("calibration stopping!");
      delay(2000);
      CalibrationCounter = CalibrationCounter + 1;
    }
    else
    {
      CalibrationCounter = CalibrationCounter + 1;
      Serial2.print("CalibrationCounter = ");
      Serial2.println(CalibrationCounter);
    }
  }
  // GY-271のキャリブレーション終了
  
  // for HY-SRF05
  pinMode( echoPin, INPUT );
  pinMode( trigPin, OUTPUT );

  //for servomoter
  servo1.init(19,4);

  // for parachute
  pinMode(cutparac, OUTPUT);      // 切り離し用トランジスタの出力宣言
  digitalWrite(cutparac, LOW);    // 切り離し用トランジスタの出力オフ

 delay(10000);
 Serial2.println("WARNING: 9v voltage on");
 digitalWrite(cutparac, HIGH); //オン
 delay(outputcutsecond*1000);//電流を流す
 Serial2.println("9v voltage off");
 digitalWrite(cutparac, LOW); //オフ
 
 stoppage();
 Serial2.println("rotate for 180°");
 servo1.write(10);// 引数は角度(°)(角度=0とすると動作が不安定になる)
 stoppage();
 delay(3000);
 servo1.write(180);
 stoppage();
 delay(3000);
 servo1.write(10);

 Serial2.println("forward");
 forward();
 delay(5000);
 stoppage();
 Serial2.println("rotating");
 rotating();
 delay(5000);
 stoppage();
 Serial2.println("reverse_rotating");
 reverse_rotating();
 delay(5000);
 stoppage();
  
} // setup関数閉じ

void loop()
{
  if (Serial1.available() > 0) // "Serial1.available() > 0"がないとGPS使えない
  { 
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) // この中でGPSのデータを使わないと，うまく値が表示されない
    { 
        // センサー値取得
        // for BME280
        Temperature = bme.readTemperature();
        Pressure = bme.readPressure() / 100.0;
        Humid = bme.readHumidity();

        // for MPU6050
        mySensor.accelUpdate();
        if (mySensor.accelUpdate() == 0)
        {
          accelX = mySensor.accelX() + 0;
          accelY = mySensor.accelY() + 0;
          accelZ = mySensor.accelZ() + 0;
          accelSqrt = mySensor.accelSqrt();
        }

        // for GPS
        gps_latitude = gps.location.lat();
        gps_longitude = gps.location.lng();       
        gps_time = gps.time.value();
        gps_velocity = gps.speed.mps();
        
        // for HY-SRF05
        digitalWrite(trigPin, LOW); 
        delay(2); 
        digitalWrite( trigPin, HIGH ); // 超音波を10ms間送信
        delay(10);
        digitalWrite( trigPin, LOW ); // 超音波を停止      
        Duration = pulseIn( echoPin, HIGH ); // センサからの入力
        if (Duration > 0) {
          Duration = Duration/2; //往復距離を半分にする
          ultra_distance = Duration*340*100/1000000; // 音速を340m/sに設定
        }

        // for GY-271
        Sum_headingDegrees = 0.0; // Sum_headingDegreesの初期化
        for (int i = 0; i < 15; i++){
          delay(10);
          Vector norm = compass.readNormalize();
          heading = atan2(norm.YAxis, norm.XAxis);
          declinationAngle = (-7.0 + (46.0 / 60.0)) / (180 / PI);
          heading += declinationAngle;
          if (heading < 0){
            heading += 2 * PI;
          }
          if (heading > 2 * PI){
            heading -= 2 * PI;
          }
          // Convert to degrees
          headingDegrees = heading * 180 / M_PI;
          if (headingDegrees < 0){
            headingDegrees += 360;
          }
          if (headingDegrees > 360){
            headingDegrees -= 360;
          }
          Sum_headingDegrees += headingDegrees;
        }
        Angle_gy271 = Sum_headingDegrees / 15;     

        // for ガイガーカウンタ
        while(Serial.available()){
          char data = Serial.read();
          Serial2.print(data);
          SensorData.print(data);
        }
        SensorData.flush(); 
    if(phase_state != 3)
    {
      Serial2.println("Press the 'p','s' or 'm' key!");
    }
    phase_state = 3;

    // センサー値を送信
    Serial2.print(gps_time);
    Serial2.print(",");
    Serial2.print(gps_latitude, 9);
    Serial2.print(",");
    Serial2.print(gps_longitude, 9);
    Serial2.print(",");
    Serial2.print(gps_velocity);
    Serial2.print(",");
    Serial2.print(Temperature);
    Serial2.print(",");
    Serial2.print(Pressure);
    Serial2.print(",");
    Serial2.print(Humid);
    Serial2.print(",");
    Serial2.print(accelX);
    Serial2.print(",");
    Serial2.print(accelY);
    Serial2.print(",");
    Serial2.print(accelZ);
    Serial2.print(",");
    Serial2.print(Angle_gy271);
    Serial2.print(",");
    Serial2.println(ultra_distance);

    // SDカードへデータを保存
    SensorData.print(gps_time);
    SensorData.print(",");
    SensorData.print(gps_latitude, 9);
    SensorData.print(",");
    SensorData.print(gps_longitude, 9);
    SensorData.print(",");
    SensorData.print(gps_velocity);
    SensorData.print(",");
    SensorData.print(Temperature);
    SensorData.print(",");
    SensorData.print(Pressure);
    SensorData.print(",");
    SensorData.print(Humid);
    SensorData.print(",");
    SensorData.print(accelX);
    SensorData.print(",");
    SensorData.print(accelY);
    SensorData.print(",");
    SensorData.print(accelZ);
    SensorData.print(",");
    SensorData.print(Angle_gy271);
    SensorData.print(",");
    SensorData.println(ultra_distance);
    SensorData.flush();
    }
  } // "if(Serial1.available()>0)"の閉じ
} // loop関数の閉じ
