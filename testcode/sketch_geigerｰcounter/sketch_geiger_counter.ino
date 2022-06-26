void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(38400, 5, 22);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial1.available()){
    char data = Serial1.read();
    Serial.println(data);
  }
}
