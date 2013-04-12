#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define DEBUG

#define ESC_A 9
#define ESC_B 6
#define ESC_C 5
#define ESC_D 3

#define RC_1 13
#define RC_2 12 
#define RC_3 11
#define RC_4 10
#define RC_5 8
#define RC_PWR A0

/*
  Arducopter v1.0
  
  RC Channels: 
    
    Throttle - CH3
    Pitch    - CH2
    Roll     - CH1
    Yaw      - CH4
    Aux      - CH5


*/

// MPU SETUP //

MPU6050 mpu; // mpu interface object

bool dmpReady = false; //flag
uint8_t mpuIntStatus;  //mpu statusbyte
uint8_t devStatus; //device status byte    
uint16_t packetSize; //estimatet packet size  
uint16_t fifoCount; //fifo buffer size   
uint8_t fifoBuffer[64]; //fifo buffer 

Quaternion q; //quaternion for mpu output
VectorFloat gravity; //gravity vector for ypr
float ypr[3]; // yaw pitch roll values

volatile bool mpuInterrupt = false; //interrupt flag

// RC/CONTROL SETUP

int ch1, ch2, ch3, ch4, ch5; //RC channel inputs
float ch3_smooth = 1000;
int va, vb, vc, vd; //velocities
int v_ac, v_bd; // velocity of axes

float bal_ac, bal_bd; // motor balances can vary between -100 & 100
float bal_axes; // throttle balance between axes -100:ac , +100:bd

int velocity; //global velocity

Servo a,b,c,d;

void setup(){
  
  initRC();
  initMPU();
  initESCs();
  initBalancing();
  
  #ifdef DEBUG
  
  Serial.begin(9600);
  Serial.flush();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  #endif
}

void loop(){

  getYPR();
  getInput();
  calculateVelocities();
  updateMotors();
  
}

void getYPR(){

  if(mpuInterrupt && fifoCount >= packetSize){
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount == 1024) mpu.resetFIFO(); // overflow
    
    else if(mpuIntStatus & 0x02){
    
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      fifoCount -= packetSize;
    
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    }
    
    
  
  }

}

void calculateVelocities(){

  velocity = map(ch3, 100, 200, 115, 22);
  bal_bd = floor(map(ch2, 100, 200, 100, -100)/10); //round to floor base 10
  
  Serial.println(ch3);
  
  v_ac = (abs(-100+bal_axes)/100)*velocity;
  v_bd = ((100+bal_axes)/100)*velocity;
  
  va = ((100+bal_ac)/100)*v_ac;
  vb = ((100+bal_bd)/100)*v_bd;
  
  vc = (abs((-100+bal_ac)/100))*v_ac;
  vd = (abs((-100+bal_bd)/100))*v_bd;
  
  #ifdef DEBUG
  /*
  Serial.print("A: "+String(va)+" B: "+String(vb)+" C: "+String(vc)+" D: "+String(vd));
  Serial.println("");
  Serial.print("VAC: "+String(v_ac)+" VBD: "+String(v_bd));
  Serial.println("");
  */
  #endif
  

}

void updateMotors(){

  a.write(va);
  c.write(vc);
  b.write(vb);
  d.write(vd);

}

void getInput(){
  
  ch1 = floor(pulseIn(RC_1, HIGH, 30000))/10;
  ch2 = floor(pulseIn(RC_2, HIGH, 30000))/10;
  ch3 = pulseIn(RC_3, HIGH, 30000)/10;
  ch3_smooth = smooth(ch3, .5, ch3_smooth);
  ch3 = ch3_smooth;
  ch4 = floor(pulseIn(RC_4, HIGH, 30000))/10;
  ch5 = floor(pulseIn(RC_5, HIGH, 30000))/10;
  
}

void arm(){

  a.write(22);
  b.write(22);
  c.write(22);
  d.write(22);
  
  delay(5000);

}

int smooth(int data, float filterVal, float smoothedVal){


  if (filterVal > 1){    
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void initRC(){
  pinMode(RC_PWR, OUTPUT);
  digitalWrite(RC_PWR, HIGH);
}

void initMPU(){

  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}

void initESCs(){

  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
  
  delay(100);
  
  arm();

}

void initBalancing(){

  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;

}

