int cutparac = 23;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 3;    //切り離し時の9V電圧を流す時間，単位はsecond


void setup() {

    pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
    digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ
    Serial.begin(115200);

}//setup関数閉じ


void loop() {
   delay(10000);
   Serial.print("WARNING: 9v voltage on.\n");
   digitalWrite(cutparac, HIGH); //オン
   delay(outputcutsecond*1000);//十秒間電流を流す
   Serial.print("WARNING: 9v voltage off.\n");
   digitalWrite(cutparac, LOW); //オフ

}//loop関数の閉じ
