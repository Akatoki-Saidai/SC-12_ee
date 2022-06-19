#include <DFRobot_QMC5883.h>
#include <math.h>

DFRobot_QMC5883 compass;

int CalibrationCounter = 1;
int calibration = 1;
double heading = 0;
double headingDegrees = 0;
double declinationAngle = 0;

void setup(){
  // put your setup code here, to run once:
    // for GY-271
  while (!compass.begin()){
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }

  if (compass.isHMC())
  {
    Serial.println("Initialize HMC5883");
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_15HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
  }
  else if (compass.isQMC())
  {
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
  Serial.println("please rotate!!!!");
  while (CalibrationCounter < 551){
      Vector norm = compass.readNormalize();
      //rotating(); // testcodeでは手動で回す．
      if (CalibrationCounter == 550)
      {
        //stoppage();
        Serial.println("calibration stopping!");
        delay(2000);
        calibration = 2;
        CalibrationCounter = CalibrationCounter + 1;
      }
      else
      {
        CalibrationCounter = CalibrationCounter + 1;
        Serial.print("CalibrationCounter = ");
        Serial.println(CalibrationCounter);
      }
    }
}
void loop() {
  // put your main code here, to run repeatedly:
  Vector norm = compass.readNormalize();
  heading = atan2(norm.YAxis, norm.XAxis);
  declinationAngle = (-7.0 + (46.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;
  if (heading < 0)        heading += 2 * PI;
  if (heading > 2 * PI){
    heading -= 2 * PI;
    // Convert to degrees
    headingDegrees = heading * 180 / M_PI;
  }
  if (headingDegrees < 0)   headingDegrees += 360;
  if (headingDegrees > 360) headingDegrees -= 360;
  Serial.println(headingDegrees);
}
