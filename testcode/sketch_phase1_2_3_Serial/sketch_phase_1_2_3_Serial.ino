// for calculation
#include <math.h>
#define rad2deg(a) ((a) / M_PI * 180.0) /* rad を deg に換算するマクロ関数 */
#define deg2rad(a) ((a) / 180.0 * M_PI) /* deg を rad に換算するマクロ関数 */

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
double delta_lng,distance,azimuth;
// you need to set up variables at first
double GOAL_lat = 35.860576667;
double GOAL_lng = 139.606960000; 

// for GY-271
#include <DFRobot_QMC5883.h>
DFRobot_QMC5883 compass;
int CalibrationCounter = 1; // キャリブレーションで取得したデータの個数
double declinationAngle; // 補正用
double heading; // 弧度法
double headingDegrees; // 度数法
double Sum_headingDegrees; // 連続15個のheadingDegreesの和
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

//for Geiger counter
int A_num1 = 0;
int A_num2 = 0;

// センサー値を格納するための変数
float Temperature, Pressure, Humid; // for BME280
double accelX, accelY, accelZ, gyroX, gyroY, gyroZ, accelSqrt; // for MPU6050
double gps_latitude, gps_longitude, gps_velocity; // for GPS
int gps_time; // for GPS
int ultra_distance = 0; // for HY-SRF05
char data; // for ガイガーカウンタ
int i = 0;

// for motor
//ゆっくり加速
void accel()
{
  for (int i = 0; i < 255; i = i + 5)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(80); // accelではdelay使う
  }
}
//早めに加速
void accel_fast()
{
  for (int i = 0; i < 255; i = i + 5)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(10); // accelではdelay使う
  }
}
// 前進
void forward()
{
  ledcWrite(0, 0); // channel, duty
  ledcWrite(1, 255);
  ledcWrite(2, 0);
  ledcWrite(3, 255);
}
//ゆっくり停止
void brake()
{
  for (int i = 255; i > 0; i = i - 5)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(80); // stoppingではdelay使う
  }
}
//早めに停止
void brake_fast()
{
  for (int i = 255; i > 0; i = i - 5)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(10); // stoppingではdelay使う
  }
}
// 停止
void off()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}
//ゆっくり回転
void slow_rotating()
{
  ledcWrite(0, 0);
  ledcWrite(1, 100);
  ledcWrite(2, 100);
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

// for phase
int phase = 1;// 走行試験では分離，遠距離，中距離，近距離
int phase_state = 0;
unsigned long currentMillis;

// for phase1
int mode_comparison = 0;
int mode_to_bmp = 0;
int count1 = 0;
int count3 = 0;
double altitude_sum_bmp = 0;
double altitude,previous_altitude,current_altitude;
unsigned long previous_millis,current_millis;

// for phase2
int type = 1;
int yeah = 1;
int type_state = 0;
double time3_2; // 時間に関するもの
double Accel[6];                  // 計測した値をおいておく変数
double Altitude[6];               // 高度
double Preac, differ1, Acsum, Acave, RealDiffer1;
double Preal, differ2, Alsum, Alave, RealDiffer2;
int i_3 = 0;
int j_3 = 0;
double RealDiffer;

// for phase3
int cutparac = 23;          // 切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 10;    // 切り離し時の9V電圧を流す時間，単位はsecond

// for phase4
double desiredDistance = 10.0; //遠距離フェーズから中距離フェーズに移行する距離
double CurrentDistance;
double Angle_Goal,rrAngle,llAngle;

// for phase5
double sum_latitude,sum_longitude;
unsigned long nowmillis;
int sum_count = 0;
int EPSILON = 30;

// for phase6
double current_distance,previous_distance,distance1,distance2;
unsigned long current_Millis,time1,time2;
int phase_5 = 1;
int count = 0;
int accel_count = 1;

// for phase7


// 緯度経度から距離を返す関数
double CalculateDis(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{

  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  double EarthRadius = 6378.137;

  // 目標地点までの距離を導出
  delta_lng = GOAL_lng - gps_longitude;

  distance = EarthRadius * acos(sin(gps_latitude) * sin(GOAL_lat) + cos(gps_latitude) * cos(GOAL_lat) * cos(delta_lng)) * 1000;

  return distance;
}

// 角度計算用の関数
double CalculateAngle(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{

  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  // 目標地点までの角度を導出
  delta_lng = GOAL_lng - gps_longitude;
  azimuth = rad2deg(atan2(sin(delta_lng), cos(gps_latitude) * tan(GOAL_lat) - sin(gps_latitude) * cos(delta_lng)));

  if (azimuth < 0)// azimuthを0°～360°に収める
  {
    azimuth += 360;
  }
  else if (azimuth > 360)
  {
    azimuth -= 360;
  }
  return azimuth;
}

void setup()
{
  // for GPS
  Serial1.begin(115200, SERIAL_8N1, 5, 18); // ESP32-GPS間のシリアル通信開始
  
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
  Serial.begin(115200); // ESP32-twelite間のシリアル通信開始
  Serial.println("TESTING:SERIAL COMMUNICATION");

  // for ガイガーカウンタ
//  Serial.begin(38400, SERIAL_8N1,34,2); // ESP32-ガイガーカウンタ間のシリアル通信開始
  // for MPU6050
  mySensor.beginAccel(ACC_FULL_SCALE_4_G);
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
  
  // for HY-SRF05
  pinMode( echoPin, INPUT );
  pinMode( trigPin, OUTPUT );

  // for servomoter
  servo1.init(19,4);

  // for parachute
  pinMode(cutparac, OUTPUT);      // 切り離し用トランジスタの出力宣言
  digitalWrite(cutparac, LOW);    // 切り離し用トランジスタの出力オフ
  
} // setup関数閉じ

void loop()
{
//  if (Serial1.available() > 0) // "Serial1.available() > 0"がないとGPS使えない
//  { 
    char c = Serial1.read();
    gps.encode(c);
//    if (gps.location.isUpdated()) // この中でGPSのデータを使わないと，うまく値が表示されない
    { 
        //現在時刻を保存
        currentMillis = millis();
        
        // センサー値取得
        // for BME280
        Temperature = bme.readTemperature();
        Pressure = bme.readPressure() / 100.0;
        Humid = bme.readHumidity();
        altitude = ( ( pow( 1013.25 /Pressure , 1/5.257) - 1 ) * ( Temperature + 273.15 ) ) / 0.0065;

        // for MPU6050
        mySensor.accelUpdate();
        if (mySensor.accelUpdate() == 0)
        {
          accelX = mySensor.accelX();
          accelY = mySensor.accelY();
          accelZ = mySensor.accelZ();
          accelSqrt = mySensor.accelSqrt();
          if(fabs(accelZ)>4.0){
            ESP.restart();
          }
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
          Duration = Duration/2; // 往復距離を半分にする
          ultra_distance = Duration*(331.5+0.6*Temperature)*100/1000000; // 音速を340m/sに設定
        } 

        // 各フェーズごとの記述
        switch(phase)
        {
          //########## 待機フェーズ ##########
          case 1:
            if(phase_state != 1)
            {
              Serial.println("Phase1: transition completed");
              // LogDataの保存
              CanSatLogData.println(gps_time);
              CanSatLogData.println("Phase1: transition completed");
              CanSatLogData.flush();

              //台の向きを横向きにする
              servo1.write(45);

              phase_state = 1;
            }

            if (mode_to_bmp == 0)
            {
              if (accelZ < -2)
              { //落下開始をまずMPUで判定
                count1++;
                if (count1 == 1)
                {
                  count1 = 0;
                  CanSatLogData.println(gps_time);
                  CanSatLogData.println("FALL STARTED(by MPU)\n");
                  CanSatLogData.flush();
                  mode_to_bmp = 1;
                }
              }
            }
            else
            {
              switch (mode_comparison)
              { //落下開始をBMPで判定
                case 0:
                  previous_millis = millis();
                  altitude_sum_bmp += altitude;
                  count3++;
                  if (count3 == 5)
                  {
                    previous_altitude = altitude_sum_bmp / 5;
                    altitude_sum_bmp = 0;
                    count3 = 0;
                    mode_comparison = 1;
                  }
                  break;

                case 1: // 500ms後
                  current_millis = millis();
                  if (current_millis - previous_millis >= 500)
                  {
                    altitude_sum_bmp += altitude;
                    count3++;
                    if (count3 == 5)
                    {
                      current_altitude = altitude_sum_bmp / 5;
                      CanSatLogData.println("current_altitude - previous_altitude = \n");
                      CanSatLogData.println(current_altitude - previous_altitude);
                      if (current_altitude - previous_altitude <= -1.0)
                      {
                        CanSatLogData.println("FALL STARTED(by BMP)\n");
                        CanSatLogData.flush();
                        phase = 2;
                      }
                      else
                      {
                        altitude_sum_bmp = 0;
                        count3 = 0;
                        mode_comparison = 0;
                      }
                    }
                  }
                  break;
              }
            }
            
            break;

          //########## 落下フェーズ ##########
          case 2:
            if(phase_state != 2)
            {
              Serial.println("Phase2: transition completed");
              // LogDataの保存
              CanSatLogData.println(gps_time);
              CanSatLogData.println("Phase2: transition completed");
              CanSatLogData.flush();

              i_3 = 0;
              j_3 = 0;
              Preac = 0; // 1個前の加速度を記憶
              Preal = 0;
              differ1 = 0.1;   // 加速度　移動平均の差
              differ2 = 0.5;   // 高度　　移動平均の差
              Acave = 0;       // 加速度　5個の平均値
              Alave = 0;       // 高度　　5個の平均値
              Acsum = 0;       // 加速度　5個の合計値
              Alsum = 0;       // 高度　　5個の合計値
              RealDiffer1 = 0; // 1秒前との差を記憶する
              RealDiffer2 = 0;

              phase_state = 2;
            }
            if (yeah == 1)
            { // データを初めから五個得るまで
              Accel[i_3] = accelSqrt;
              Altitude[i_3] = altitude;
              i_3 = i_3 + 1;
              if (i_3 == 6)
              { // 5個得たらその時点での平均値を出し，次のフェーズへ
                yeah = 2;
                i_3 = 0; // iの値をリセット
                for (j_3 = 1; j_3 < 6; j_3++)
                { // j_3=0の値は非常に誤差が大きいので1から
                  Acsum = Acsum + Accel[j_3];
                  Alsum = Alsum + Altitude[j_3];
                }
                Acave = Acsum / 5;
                Alave = Alave / 5;
                time3_2 = currentMillis;
              }
            }
            else
            {
              Preac = Acave;
              Preal = Alave;
              Accel[i_3] = accelSqrt;
              Altitude[i_3] = altitude;
              for (j_3 = 0; j_3 < 5; j_3++)
              {
                Acsum = Acsum + Accel[j_3];
                Alsum = Alsum + Altitude[j_3];
              }
              Acave = Acsum / 5;
              Alave = Alsum / 5;
              RealDiffer1 = Preac - Acave;
              RealDiffer2 = Preal - Alave;
              if (i_3 == 5)
              {
                i_3 = 0;
                Acsum = 0;
                Alsum = 0;
              }
              else
              {
                i_3 = i_3 + 1;
                Acsum = 0;
                Alsum = 0;
              }
              if (currentMillis - time3_2 > 1000)
              {
                if (RealDiffer1 < differ1)
                { // 移動平均が基準以内の変化量だった時
                  phase = 3;
                }
                else if (RealDiffer2 < differ2)
                {
                  phase = 3;
                }
                else
                {
                  time3_2 = currentMillis;
                }
              }
            }
            
            break;

          //########## 分離フェーズ ##########
          case 3:
            if(phase_state != 3)
            {
              Serial.println("Phase3: transition completed");
              // LogDataの保存
              CanSatLogData.println(gps_time);
              CanSatLogData.println("Phase3: transition completed");
              CanSatLogData.flush();

              phase_state = 3;
            }

            Serial.println("WARNING: 9v voltage on");
            //digitalWrite(cutparac, HIGH); // オン
            delay(outputcutsecond*1000);// 電流を流す
            Serial.println("9v voltage off");
            digitalWrite(cutparac, LOW); // オフ

            phase = 4;

            break;

          //########## 遠距離フェーズ ##########
          case 4:
            if(phase_state != 4)
            {
              Serial.println("Phase4: transition completed");
              // LogDataの保存
              CanSatLogData.println(gps_time);
              CanSatLogData.println("Phase4: transition completed");
              CanSatLogData.flush();

              // 台の位置を調整
              if(accelZ < -0.95){
                servo1.write(10);//引数は角度(°)
                delay(1000);
              }
              else if(accelZ > 0.85){
                servo1.write(160);
                delay(1000);
              }
 
              // GY-271の初期化
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
              // GY-271の初期化終了

              // パラシュートと絡まらないように約3秒間前進
              accel();
              forward();
              delay(1000);
              brake();
              off();
              
              
              // GY-271のキャリブレーション
              delay(100);
              Serial.println("calibration rotating!");
              while (CalibrationCounter < 551)
              {
                Vector norm = compass.readNormalize();
                rotating();
                if (CalibrationCounter == 550)
                {
                  off();
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
              // GY-271のキャリブレーション終了
              
              phase_state = 4;
            }
            
            CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude); // 現在位置とゴールとの距離を計算

            if (desiredDistance >= CurrentDistance)
            {
              // カラーコーンとの距離が理想値よりも小さい場合は次のフェーズに移行する
              phase = 5;
            }else{
              delay(100);
              accel();
              forward();
              delay(500);
              brake();
              off();
              // Goalまでの偏角を計算する
              Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
  
              Sum_headingDegrees = 0.0;
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

              // どちらに回ればいいか計算
              rrAngle = -Angle_gy271 + Angle_Goal;
              if (rrAngle < 0){
                rrAngle += 360;
              }
              if (rrAngle > 360){
                rrAngle -= 360;
              }
              llAngle = Angle_gy271 - Angle_Goal;
              if (llAngle < 0){
                llAngle += 360;
              }
              if (llAngle > 360){
                llAngle -= 360;
              }
  
              if (rrAngle > llAngle){
                //反時計回り
                if (llAngle > 20){
                  rotating();
                  delay(150);
                  off();
                }
              }else{
                //時計回り
                if (rrAngle > 20){
                  reverse_rotating();
                  delay(150);
                  off();
                }
              }
            }

            break;

          //########## 中距離フェーズ ##########
          case 5:
            if(phase_state != 5)
            {
              Serial.println("Phase5: transition completed");
              // LogDataの保存
              CanSatLogData.println(gps_time);
              CanSatLogData.println("Phase5: transition completed");
              CanSatLogData.flush();

              phase_state = 5;
            }
            
            nowmillis = millis(); //現在時刻をnowmillisに保存

            //初期化
            sum_latitude = 0.0; 
            sum_longitude = 0.0;
            sum_count = 0;
            
            while(millis() - nowmillis < 3000){
              // GPS
              char c = Serial1.read(); // GPSチップからのデータを受信
              gps.encode(c);           // GPSチップからのデータの整形
              sum_latitude += gps.location.lat();
              sum_longitude += gps.location.lng();       
              sum_count++;
            }
            gps_latitude = sum_latitude/sum_count;
            gps_longitude = sum_longitude/sum_count;
  
            CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
            
            if (1.3 >= CurrentDistance || (ultra_distance < 300 && ultra_distance != 0)){
              // カラーコーンとの距離が理想値よりも小さい場合は次のフェーズに移行する
              phase = 6;
            }else{
              Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
  
              // ゆっくり回転開始
              slow_rotating();
  
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

              // ゴール方向を向くまで回転
              while(fabs(headingDegrees - Angle_Goal)>10){
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
              }
              
              //回転停止
              off();
  
              //少し進む
              delay(100);
              accel();
              forward();
              delay(500);
              brake();
              off();
            }
            break;


          //########## 近距離フェーズ ##########
          case 6:
            if(phase_state != 6)
            {
              Serial.println("Phase6: transition completed");
              // LogDataの保存
              CanSatLogData.println(gps_time);
              CanSatLogData.println("Phase6: transition completed");
              CanSatLogData.flush();

              phase_state = 6;
            }
            current_distance = ultra_distance;

            switch (phase_5)
            {

              case 1:
                current_Millis = millis();
                
                Serial.print("phase(rotating)=");
                Serial.println(phase_5);
                CanSatLogData.println("-----------------------");
                CanSatLogData.println("ROTATING PHASE");
                CanSatLogData.print("GPS TIME:");
                CanSatLogData.println(gps_time);
                CanSatLogData.println("searching for the goal...");
                CanSatLogData.flush();
                
                delay(100);
                //ゆっくり回転開始
                slow_rotating();
                
                if (ultra_distance < 600 && ultra_distance != 0)
                {
                  phase_5 = 2;
                  //何か物体を検知したら回転停止
                  delay(1100);
                  off();
                  Serial.println("STOP!");
                  CanSatLogData.println("-----------------------");
                  CanSatLogData.println("STOP PHASE");
                  CanSatLogData.print("GPS TIME:");
                  CanSatLogData.println(gps_time);
                  CanSatLogData.println("goal is detected!");
                  CanSatLogData.flush();
                }          
                break;

              case 2:
                Serial.print("phase(check_former)=");
                Serial.println(phase_5);
                if (abs(current_distance - previous_distance) < EPSILON)
                {
                  count++;
                }
                if (count == 5)
                { //ノイズチェック
                  phase_5 = 3;
                  time1 = millis();
                  distance1 = current_distance;
                  count = 0;
                  forward();
                }
                previous_distance = current_distance;
    
                break;
    
              case 3:
                Serial.print("phase(forward)=");
                Serial.println(phase_5);
                CanSatLogData.println("-----------------------");
                CanSatLogData.println("FORWARD PHASE");
                CanSatLogData.print("GPS TIME:");
                CanSatLogData.println(gps_time);
                CanSatLogData.println("moving for 1000[ms]");
                CanSatLogData.flush();
                time2 = millis();
                if(accel_count != 0){
                  accel_fast();
                }
                accel_count = 0;
                forward();
                if (time2 - time1 >= 100) //約1秒間前進したら
                {
                  brake_fast();
                  off();
                  accel_count = 1;
                  phase_5 = 4;
                }
                break;
    
              case 4:
                Serial.print("phase(check_later)=");
                Serial.println(phase_5);
                if (abs(current_distance - previous_distance) < EPSILON)
                {
                  count++;
                }
                if (count == 5)
                { //ノイズチェック
                  phase_5 = 5;
                  distance2 = current_distance;
                  count = 0;
                }
                previous_distance = current_distance;
                break;
    
              case 5:
                Serial.print("phase(judge)=");
                Serial.println(phase_5);
                CanSatLogData.println("-----------------------");
                CanSatLogData.println("JUDGEING PHASE");
                CanSatLogData.print("GPS TIME:");
                CanSatLogData.println(gps_time);
                CanSatLogData.flush();
                
                if (distance2 - distance1 < 0)
                {
                  if (distance2 < 100)
                  {
                    phase_5 = 6;
                    CanSatLogData.println("goal!");
                    CanSatLogData.flush();
                  }
                  else
                  {
                    phase_5 = 3;
                    CanSatLogData.println("approaching!");
                    CanSatLogData.flush();
                  }
                }
                else
                {
                  phase_5 = 1;
                  CanSatLogData.println("receding...");
                  CanSatLogData.flush();
                }
                break;
    
              case 6:
                Serial.print("phase(goal)=");
                Serial.println(phase_5);
                Serial.println("GOAL!");
                CanSatLogData.println("-----------------------");
                CanSatLogData.println("GOAL PHASE");
                CanSatLogData.print("GPS TIME:");
                CanSatLogData.println(gps_time);
                CanSatLogData.println("conglatulations!");
                phase_5 = 7;
                break;
    
              case 7:
                off();
                break;

            }
            
         /* //########## スタックフェーズ ##########
          case 8:
            if(phase_state != 8)
            {
              Serial.println("Phase7: transition completed");
              // LogDataの保存
              CanSatLogData.println(gps_time);
              CanSatLogData.println("Phase7: transition completed");
              CanSatLogData.flush();

              phase_state = 8;
            }

            
            break;*/    
        }

        /*// センサー値を送信
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
        Serial.print(Angle_gy271);
        Serial.print(",");
        Serial.println(ultra_distance);*/
    
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
       
        // for ガイガーカウンタ
//        A_num1 = 0;
//        A_num2 = 0;
//        while(Serial.available()){
//          char data = Serial.read();
//          if(data == 'A'){
//            A_num1 = 1;
//          }
//          if(A_num1 > 0){
//            A_num1 = A_num1 + 1;
//            if(A_num1 > 12 && A_num2 < 1){
//              SensorData.print(data);
//            }
//            if(data == 'h'){
//              A_num2 = 1;
//            }
//          }
//        }
        SensorData.flush();
    }
  } // "if(Serial1.available()>0)"の閉じ
//} // loop関数の閉じ
