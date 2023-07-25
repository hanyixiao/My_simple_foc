#include "MPU6050_tockn.h"
#include <SimpleFOC.h>

MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

MPU6050 mpu6050(I2Ctwo);

void setup()
{
  Serial.begin(115200);
  _delay(750);

  // 针对最新版本ESP-Arduino 2.0.2,采用下面两句
  I2Cone.begin(19, 18, 400000UL); // SDA0,SCL0
  I2Ctwo.begin(23, 5, 400000UL);

  sensor0.init(&I2Cone);
  // sensor1.init(&I2Ctwo);
  // do{
  //   mpu6050.begin();
  // }while(!mpu6050.isReady());

  // Serial.print("mpu6050 init success");
  Serial.print('\t');
  
  // mpu6050.calcGyroOffsets(true);
  pinMode(32, OUTPUT); // 将数字引脚2设置为输出模式 
  pinMode(33, OUTPUT); 
  pinMode(25, OUTPUT); 
  pinMode(26, OUTPUT); 
  pinMode(27, OUTPUT); 
  pinMode(14, OUTPUT); 
  
  digitalWrite(32, LOW); // 输出低电平
  digitalWrite(33, LOW);
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
  digitalWrite(14, LOW);
}

void loop()
{
  // mpu6050.update();
  // 角度读取
  // double mpu_yaw = mpu6050.getAngleZ();   // tockn的getangle，通过一阶置信计算
  // double mpu_pitch = mpu6050.getAngleY(); // tockn的getangle，通过一阶置信计算
  // double mpu_roll = mpu6050.getAngleX();  // tockn的getangle，通过一阶置信计算
  // // 加速度读取
  // //    double accX = mpu6050.getAccX();
  // //    double accY = mpu6050.getAccY();
  // //    double accZ = mpu6050.getAccZ();
  // Serial.print("mpu_roll:");
  // Serial.print(mpu_roll);
  // Serial.print('\t');
  // Serial.print("mpu_pitch:");
  // Serial.print(mpu_pitch);
  // Serial.print('\t');
  // Serial.print("mpu_yaw:");
  // Serial.println(mpu_yaw);
  // Serial.print('\t');
  //_delay(200);
  sensor0.update();
  // sensor1.update();

  Serial.print("sensor0:");
  Serial.print(sensor0.getAngle());
  Serial.print('\t');
  // Serial.print("sensor1:");
  // Serial.print(sensor1.getAngle());
  Serial.print('\t');
  delay(100);
}
