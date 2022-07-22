int cutparac = 23;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 3;    //切り離し時の9V電圧を流す時間，単位はsecond
int i = 10;

void setup() {
  // put your setup code here, to run once:
  pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
  digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ
  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8N1,34,2);//RX,TX
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial1.available()){
    char data = Serial1.read();
    Serial.print(data);
  }

  Serial.println("9v voltage will be output in 10 seconds");
  while(i!=0){
    Serial.println(i);
    delay(1000);
    i--;      
  }
  i = 10;
 
  Serial.print("WARNING: 9v voltage on.\n");
  digitalWrite(cutparac, HIGH); //オン
  delay(outputcutsecond*1000);//十秒間電流を流す
  Serial.print("WARNING: 9v voltage off.\n");
  digitalWrite(cutparac, LOW); //オフ
 
}
