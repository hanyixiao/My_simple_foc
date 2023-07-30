#include "MPU6050_tockn.h"
#include <SimpleFOC.h>

#include "AS5600.h"

#include "DengFOC.h"

int Sensor_DIR=1;    //传感器方向
int Motor_PP=7;    //电机极对数

void setup() {
  Serial.begin(115200);
  DFOC_Vbus(12.6);   //设定驱动器供电电压
  DFOC_alignSensor(Motor_PP,Sensor_DIR);
}

void loop() 
{
  float Kp=0.133;
  float Sensor_Angle=DFOC_M0_Angle();
  Serial.printf("target angle %f read angle %f\r\n",serial_motor_target(),DFOC_M0_Angle());
  setTorque(Kp*(serial_motor_target()-Sensor_DIR*Sensor_Angle)*180/PI,_electricalAngle());   //位置闭环
  serialReceiveUserCommand();   //接收串口指令
}