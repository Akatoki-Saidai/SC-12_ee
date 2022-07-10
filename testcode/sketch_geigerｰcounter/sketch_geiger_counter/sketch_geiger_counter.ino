void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200, SERIAL_8N1,34,35);
  Serial1.begin(38400, SERIAL_8N1,5,18);//RX,TX
  Serial2.begin(115200, SERIAL_8N1,16,17);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()){
    char data = Serial.read();
    Serial.print(data);
  }
}
