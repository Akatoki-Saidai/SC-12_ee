/*!
 * @file QMC5883_compass.cpp
 * @brief The program shows how to realize the function compass.When the program runs, please spin QMC5883 freely to accomplish calibration.
 * @n 3-Axis Digital Compass IC
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2017
 * @copyright	GNU Lesser General Public License
 *
 * @author [dexian.huang](952838602@qq.com)
 * @version  V1.0
 * @date  2017-7-3
 */

#include <Wire.h>
#include <DFRobot_QMC5883.h>

//void DFRobot(){
  DFRobot_QMC5883 compass;
//}

int n1 = 0;
int n2 = 0;

void setup()
{
  Serial.begin(115200);

  /*
  while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }

    if(compass.isHMC()){
        Serial.println("Initialize HMC5883");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
    }
   else if(compass.isQMC()){
        Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
   }*/
  }
void loop()
{
//DFRobot();
  if(n1 < 500){

    //================= 仮の待機時間 ==================
    //一回だけ実行
    if(n2 == 0){
      Serial.println("===Waiting time===");
      n2 = 1;
    }
    
    Serial.print("n = ");
    Serial.println(n1);
    n1 = n1 + 1;
    delay(10);
  }
  //=================地磁気使用開始====================
  else{
    if(n2 == 1){
    while (!compass.begin()){
      Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
      delay(500);
    }
    if(compass.isHMC()){
      Serial.println("Initialize HMC5883");
      compass.setRange(HMC5883L_RANGE_1_3GA);
      compass.setMeasurementMode(HMC5883L_CONTINOUS);
      compass.setDataRate(HMC5883L_DATARATE_15HZ);
      compass.setSamples(HMC5883L_SAMPLES_8);
    }
     else if(compass.isQMC()){
       Serial.println("Initialize QMC5883");
       compass.setRange(QMC5883_RANGE_2GA);
       compass.setMeasurementMode(QMC5883_CONTINOUS); 
       compass.setDataRate(QMC5883_DATARATE_50HZ);
       compass.setSamples(QMC5883_SAMPLES_8);
     }
     n2 = 2;
    }
    
    Vector norm = compass.readNormalize();
  
    // Calculate heading
    float heading = atan2(norm.YAxis, norm.XAxis);
  
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    //float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    float declinationAngle = (-7.0 + (46.0 / 60.0)) / (180 / PI);
    heading += declinationAngle;
    //Serial.println(heading);
    //heading -= PI;
    //heading += PI/3;
    //heading += PI/2;
    //Serial.println(heading);
    
    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0){
      heading += 2 * PI;
    }
  
    if (heading > 2 * PI){
      heading -= 2 * PI;
    }
  
    // Convert to degrees
    float headingDegrees = heading * 180/M_PI; 
  
  
  
    //Serial.print("Heading = ");
    //Serial.print(heading);
    //Serial.print(",");
    //Serial.println(headingDegrees);
    //Serial.println();
  
    //delay(10);
  }
}
