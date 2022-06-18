#include <TinyGPS++.h>
TinyGPSPlus gps;


void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
}

void loop() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
    }
  }
}
