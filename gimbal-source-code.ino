/* This code was written by Recep Said Dülger
  for Xsens competition - 2020  
  ElectroGtu Team
  Reference Website : https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
  These library was used :    I2Cdev.h   (https://github.com/jrowberg/i2cdevlib)
                              MPU6050.h  (Arduino offical website)
                              Wire.h     (Arduino offical website)
                              Servo.h    (Arduino offical website)
                              SD.h       (Arduino offical website)
                                             */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" 
#include <Servo.h>
#include <SD.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

Servo servo0;
Servo servo1;
Servo servo2;
float correct;
int j = 0;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  

bool blinkState = false;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;   
uint16_t fifoCount;    
uint8_t fifoBuffer[64]; 


Quaternion q;           // [w, x, y, z]         
VectorInt16 aa;         // [x, y, z]           
VectorInt16 aaReal;     // [x, y, z]        
VectorInt16 aaWorld;    // [x, y, z]          
VectorFloat gravity;    // [x, y, z]            
float euler[3];         // [psi, theta, phi]    
float ypr[3];           // [yaw, pitch, roll]  


uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/* INTERRUPT */       
volatile bool mpuInterrupt = false;    
void dmpDataReady() {
  mpuInterrupt = true;
}            

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  //Serial.begin(38400);
  //while (!Serial); 
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551);

  if (devStatus == 0) {
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // Serial.print(F("DMP Initialization failed (code "));
  }
  servo0.attach(10);
  servo1.attach(9);
  servo2.attach(8);
}

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
  } 
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
    
    if (j <= 300) {
      correct = ypr[0]; 
      j++;
    }
    else {
      ypr[0] = ypr[0] - correct; 
      int servo0Value = map(ypr[0], -90, 90, 0, 180);
      int servo1Value = map(ypr[1], -90, 90, 0, 180);
      int servo2Value = map(ypr[2], -90, 90, 180, 0);
      
      servo0.write(servo0Value);
      servo1.write(servo1Value);
      servo2.write(servo2Value);
    }
#endif
  }
}
