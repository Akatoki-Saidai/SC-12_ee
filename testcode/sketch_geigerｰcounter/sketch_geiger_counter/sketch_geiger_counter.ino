void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8N1,34,12);//RX,TX
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial1.available()){
    char data = Serial1.read();
    Serial.print(data);
  }
}
