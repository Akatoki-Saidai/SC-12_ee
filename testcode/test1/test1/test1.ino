const unsigned int TRIG_PIN=13;
const unsigned int ECHO_PIN=12;
long duration;
int distance;

void setup() 
{
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() 
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  distance= duration*0.034/2;

  if(duration==0)
  {
     Serial.println("No pulse from sensor");
  } 
  else
  {
     Serial.print("Distance :");
     Serial.print(distance);
     Serial.println(" cm");
  }
  delay(1000);
}
