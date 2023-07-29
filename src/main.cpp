#include "MPU6050_tockn.h"
#include <SimpleFOC.h>
#include "DengFOC.h"
#include "AS5600.h"
extern Sensor_AS5600 S0;

void setup() {
  Serial.begin(115200);  
  DFOC_Vbus(12.6);   
}

void loop() 
{

  //输出角度值
  Serial.print("angle:");
  S0.Sensor_update();
  Serial.println(DFOC_M0_Angle());  
}