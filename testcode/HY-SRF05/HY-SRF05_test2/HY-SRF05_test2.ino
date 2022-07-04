/*
 * ESP-WROOM-02/32
 */
 
#define echoPin 35 // Echo Pin
#define trigPin 4 // Trigger Pin
 
double Duration = 0; //受信した間隔
double Distance = 0; //距離
 
void setup() {
Serial.begin( 115200 );//Arduino Unoは9600,ESPで試す場合は115200
pinMode( echoPin, INPUT );
pinMode( trigPin, OUTPUT );
}
 
void loop() {
  digitalWrite(trigPin, LOW); 
  delay(2); 
  digitalWrite( trigPin, HIGH ); //超音波を出力
  delay(10); //
  digitalWrite( trigPin, LOW );
  
  Duration = pulseIn( echoPin, HIGH ); //センサからの入力

  if(Duration==0){
    Serial.println("0");
  }
  if (Duration > 0) {
    Duration = Duration/2; //往復距離を半分にする
    Distance = Duration*340*100/1000000; // 音速を340m/sに設定
    Serial.print("Distance:");
    Serial.print(Distance);
    Serial.println(" cm");
  }
  delay(100);//取得間隔0.1秒
}
