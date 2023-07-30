#include "MPU6050_tockn.h"
#include <SimpleFOC.h>

#include "AS5600.h"

#include "DengFOC.h"

int Sensor_DIR=1;    
int Motor_PP=7;    
int pin = 0;

void setup() {
  Serial.begin(115200);
  DFOC_Vbus(12.6);   
  DFOC_alignSensor(Motor_PP,Sensor_DIR);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() 
{
  float Sensor_Vel;
  digitalWrite(LED_BUILTIN,pin);
  pin=!pin;
  DFOC_M0_SET_ANGLE_PID(0.5,0,0,0);
  DFOC_M0_SET_VEL_PID(3.1,0.05,0.1,100);
  Sensor_Vel=DFOC_M0_Velocity();
  DFOC_M0_setVelocity(0.5);
  // DFOC_M0_set_Velocity_Angle(serial_motor_target()); 
  // Serial.printf("speed %f read speed%f\r\n",2.0f,Sensor_Vel);
}