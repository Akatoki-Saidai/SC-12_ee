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
#include <Wire.h> //I2C通信
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

// for GPS
#include <TinyGPS++.h>
TinyGPSPlus gps;

// for GY-271
#include <DFRobot_QMC5883.h>
DFRobot_QMC5883 compass;
int CalibrationCounter = 1; //キャリブレーションで取得したデータの個数
double declinationAngle; //補正用
double heading; //弧度法
double headingDegrees; //度数法
double Sum_headingDegrees; //連続15個のheadingDegreesの和
double Angle_gy271; //連続15個の平均値(度数法)

// for SD Card
#include <SPI.h> //SDカードはSPI通信
#include <SD.h>
File CanSatLogData;
File SensorData;
const int sck = 13, miso = 15, mosi = 14, ss = 27;

// for HY-SRF05
#define echoPin 16 // Echo Pin
#define trigPin 17 // Trigger Pin
double Duration = 0; //受信した間隔

//センサー値を格納するための変数
double Temperature, Pressure, Humid; // for BME280
double accelX, accelY, accelZ, gyroX, gyroY, gyroZ, accelSqrt; // for MPU6050
double gps_latitude, gps_longitude, gps_velocity; // for GPS
int gps_time; // for GPS
double ultra_distance = 0; // for HY-SRF05

//for motor
//前進
void forward()
{
  ledcWrite(0, 0); // channel, duty
  ledcWrite(1, 127);
  ledcWrite(2, 0);
  ledcWrite(3, 107);
}
//後転
void back()
{
  ledcWrite(0, 0);
  ledcWrite(1, 127);
  ledcWrite(2, 0);
  ledcWrite(3, 127);
}
//停止
void stoppage()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}
//回転
void rotating()
{
  ledcWrite(0, 0);
  ledcWrite(1, 117);
  ledcWrite(2, 117);
  ledcWrite(3, 0);
}
//反回転
void reverse_rotating()
{
  ledcWrite(0, 127);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 127);
}

//for parachute
int cutparac = 23;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 3;    //切り離し時の9V電圧を流す時間，単位はsecond
int phase_state = 0;
char key = '0';

void setup()
{
  //for GPS
  Serial1.begin(115200, SERIAL_8N1, 5, 18); //ESP32-GPS間のシリアル通信開始
  
  // SD Card initialization
  SPI.begin(sck, miso, mosi, ss);
  SD.begin(ss, SPI);
  CanSatLogData = SD.open("/CanSatLogData.txt", FILE_APPEND);
  SensorData = SD.open("/SensorData.txt", FILE_APPEND);
  CanSatLogData.println("START_RECORD");
  CanSatLogData.flush();
  SensorData.println("gps_time,gps_latitude,gps_longitude,gps_velocity,Temperature,Pressure,Humid,accelX,accelY,accelZ,headingDegrees,Angle_gy271,ultra_distance");
  SensorData.flush();

  //for Serial communication
  Serial.begin(115200); //PC-ESP32間のシリアル通信開始
  
  // for MPU6050
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  
  // for BME280
  bool status;
  status = bme.begin(0x76);
  delay(1000);

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
  //GY-271の初期化
  while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }
  if (compass.isHMC())
  {
    Serial.println("Initialize HMC5883");
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_15HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
  }
  else if (compass.isQMC())
  {
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
  //GY-271の初期化終了
  
  //GY-271のキャリブレーション
  delay(100);
  Serial.println("calibration rotating!");
  while (CalibrationCounter < 551)
  {
    Vector norm = compass.readNormalize();
    rotating(); // testcodeでは手動で回す．
    if (CalibrationCounter == 550)
    {
      stoppage();
      Serial.println("calibration stopping!");
      delay(2000);
      CalibrationCounter = CalibrationCounter + 1;
    }
    else
    {
      CalibrationCounter = CalibrationCounter + 1;
      Serial.print("CalibrationCounter = ");
      Serial.println(CalibrationCounter);
    }
  }
  //GY-271のキャリブレーション終了

  // for HY-SRF05
  pinMode( echoPin, INPUT );
  pinMode( trigPin, OUTPUT );

  // for parachute
  pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
  digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ
  
} // setup関数閉じ

void loop()
{
  if (Serial1.available() > 0) //"Serial1.available() > 0"がないとGPS使えない
  { 
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) //この中でGPSのデータを使わないと，うまく値が表示されない
    { 
        //センサー値取得
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
        digitalWrite( trigPin, HIGH ); //超音波を10ms間送信
        delay(10);
        digitalWrite( trigPin, LOW ); //超音波を停止      
        Duration = pulseIn( echoPin, HIGH ); //センサからの入力
        if (Duration > 0) {
          Duration = Duration/2; //往復距離を半分にする
          ultra_distance = Duration*340*100/1000000; // 音速を340m/sに設定
        }
        delay(100);//取得間隔0.1秒

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
    }
    if(phase_state != 3)
    {
      Serial.println("Press the 'p' key to cut the parachute");
    }
    phase_state = 3;

    if(Serial.available()>0)//何かキーが押されたら
    {
       key = Serial.read();
       switch(key){
        case 'p':
           Serial.println("WARNING: 9v voltage on\n");
           digitalWrite(cutparac, HIGH); //オン
           delay(outputcutsecond*3);//電流を流す
           Serial.println("9v voltage off.\n");
           digitalWrite(cutparac, LOW); //オフ
           break;
        default:
           Serial.println("The key you typed is not the 'p' key!");
           break;
       }
    }

    // シリアルモニタにセンサー値を表示
    Serial.print(gps_time);
    Serial.print(",");
    Serial.print(gps_latitude, 9);
    Serial.print(",");
    Serial.print(gps_longitude, 9);
    Serial.print(",");
    Serial.print(gps_velocity);
    Serial.print(",");
    Serial.print(Temperature);
    Serial.print(",");
    Serial.print(Pressure);
    Serial.print(",");
    Serial.print(Humid);
    Serial.print(",");
    Serial.print(accelX);
    Serial.print(",");
    Serial.print(accelY);
    Serial.print(",");
    Serial.print(accelZ);
    Serial.print(",");
    Serial.print(headingDegrees);
    Serial.print(",");
    Serial.print(Angle_gy271);
    Serial.print(",");
    Serial.println(ultra_distance);

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
    SensorData.print(headingDegrees);
    SensorData.print(",");
    SensorData.print(Angle_gy271);
    SensorData.print(",");
    SensorData.println(ultra_distance);
    SensorData.flush();
  }
} // loop関数の閉じ
