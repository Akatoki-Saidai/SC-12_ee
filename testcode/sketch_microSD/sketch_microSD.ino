#include <SD.h>

const uint8_t cs_SD = 5; // GPIO5=CS
const char* fname = "/test1.txt";    //"/ファイルの名前.text"

File fo; // ファイルオブジェクト
int i = 0;

void setup() {
  Serial.begin(115200);
  
  // SDライブラリを初期化
  SD.begin(cs_SD, SPI, 24000000, "/sd");

  // 書き込みモードでファイルを開く
  fo = SD.open(fname, FILE_WRITE);

  // "書きたいこと" 
  fo.println("test test test.");
  fo.println("Hello World!!!");
  fo.println("あいうえお");
  
  // ファイルを閉じる
  fo.close();

  Serial.println("finish writting"); 
  
}

void loop() {
  
}
