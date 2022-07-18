#include <TinyGPS++.h>
TinyGPSPlus gps;


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200,SERIAL_8N1,5,18);
}

void loop() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
    }
  }
}
