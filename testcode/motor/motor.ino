
void setup() {
  Serial.begin(115200);
  ledcSetup(0, 490, 8);
  ledcSetup(1, 490, 8);
  ledcSetup(2, 490, 8);
  ledcSetup(3, 490, 8);

  ledcAttachPin(32, 0);
  ledcAttachPin(33, 1);
  ledcAttachPin(26, 2);
  ledcAttachPin(25, 3);
  
}

void loop() {
  
  forward();
  delay(5000);
  rotating();
  delay(5000);
  reverse_rotating();
  delay(5000);
  
}

//前進
void forward(){
  ledcWrite(0, 0); //channel, duty
  ledcWrite(1, 200);
  ledcWrite(2, 200);
  ledcWrite(3, 0);
}

//回転
void rotating(){
  ledcWrite(0, 200);
  ledcWrite(1, 0);
  ledcWrite(2, 200);
  ledcWrite(3, 0);  
}
//反回転
void reverse_rotating(){
  ledcWrite(0, 0);
  ledcWrite(1, 200);
  ledcWrite(2, 0);
  ledcWrite(3, 200);  
}
