//for servomoter
#include <ESP_servo.h>
#include <stdio.h>
#include <string>

ESP_servo servo1;

void setup() {
   
    //for servomoter
    servo1.init(19,1);
    

}//setup関数閉じ


void loop() {
    servo1.write(90);//引数は角度(°)
    delay(1000);
    servo1.write(0);
    delay(1000);
}//loop関数の閉じ
