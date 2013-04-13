#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

#define DEBUG


/*  Arduino Pin configuration
 *  
 */

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


/* ESC configuration
 *
 */

#define ESC_MIN 22
#define ESC_MAX 115
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 5000

/* RC configuration
 * 
 */

#define RC_HIGH 1000
#define RC_LOW 2000

/*  PID configuration
 *  
 */

#define PITCH_P_VAL 2
#define PITCH_I_VAL 5
#define PITCH_D_VAL 1

#define ROLL_P_VAL 2
#define ROLL_I_VAL 5
#define ROLL_D_VAL 1

#define YAW_P_VAL 2
#define YAW_I_VAL 5
#define YAW_D_VAL 1


/* Flight parameters
 *
 */

#define PITCH_MIN -45
#define PITCH_MAX 45
#define ROLL_MIN -45
#define ROLL_MAX 45
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 15
#define PID_ROLL_INFLUENCE 15
#define PID_YAW_INFLUENCE 15


/*  MPU variables
 *
 */

MPU6050 mpu;                           // mpu interface object


uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values

volatile bool mpuInterrupt = false;    //interrupt flag


/*  RC variables
 *
 */

float ch1, ch2, ch3, ch4, ch5;         // RC channel inputs
float ch3_smooth = 1000;               // may be unnecessary with new input method

unsigned long rcLastChange1 = millis();
unsigned long rcLastChange2 = millis();
unsigned long rcLastChange3 = millis();
unsigned long rcLastChange4 = millis();
unsigned long rcLastChange5 = millis();

/*  Motor controll variables
 *
 */

int velocity;                         // global velocity

float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd

int va, vb, vc, vd;                   //velocities
int v_ac, v_bd;                       // velocity of axes

Servo a,b,c,d;

/*  PID variables
 *
 */

PID pitchReg(&ypr[1], &bal_bd, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &bal_ac, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&ypr[0], &bal_axes, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);


/*  Setup function
 *
 */

void setup(){
  
  initRC();                            // Self explaining
  initMPU();
  initESCs();
  initBalancing();
  initRegulators();
  
  #ifdef DEBUG                        // Device tests go here
  
  Serial.begin(9600);                 // Serial only necessary if in DEBUG mode
  Serial.flush();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  #endif
}

/* loop function
 *
 */

void loop(){
  
  while(!mpuInterrupt && fifoCount < packetSize){
     
    /* Do nothing while MPU is not working
     * This should be a VERY short period
     */
      
  }
  
  getYPR();                          
  computePID();
  calculateVelocities();
  updateMotors();
  
}

/*  computePID function
 *
 *  formats data for use with PIDLib
 *  and computes PID output
 */

void computePID(){

  ch2 = map(ch2, RC_LOW, RC_HIGH, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW, RC_HIGH, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW, RC_HIGH, YAW_MIN, YAW_MAX);
  
  ypr[0] = ypr[0] * 180/M_PI;
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;
  
  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();

}

/*  getYPR function
 *
 *  gets data from MPU and
 *  computes pitch, roll, yaw on the MPU's DMP
 */

void getYPR(){
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount == 1024){ 
      
      Serial.println("FIFO overflow!");
      mpu.resetFIFO(); 
    
    }else if(mpuIntStatus & 0x02){
    
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      fifoCount -= packetSize;
    
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    }

}

/*  calculateVelocities function
 *  
 *  calculates the velocities of every motor
 *  using the PID output
 */

void calculateVelocities(){

  velocity = map(ch3, RC_LOW, RC_HIGH, ESC_MIN, ESC_MAX);
  
  v_ac = (abs(-100+bal_axes)/100)*velocity;
  v_bd = ((100+bal_axes)/100)*velocity;
  
  va = ((100+bal_ac)/100)*v_ac;
  vb = ((100+bal_bd)/100)*v_bd;
  
  vc = (abs((-100+bal_ac)/100))*v_ac;
  vd = (abs((-100+bal_bd)/100))*v_bd;
  
  if(velocity < ESC_TAKEOFF_OFFSET){
  
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  
  }
  
  #ifdef DEBUG
  
  //Serial.print("A: "+String(va)+" B: "+String(vb)+" C: "+String(vc)+" D: "+String(vd));
  /*Serial.print("AC: ");
  Serial.print(bal_ac);
  Serial.print(" BD: ");
  Serial.print(bal_bd);
  Serial.println("");*/
  #endif
  
}

void updateMotors(){

  a.write(va);
  c.write(vc);
  b.write(vb);
  d.write(vd);

}

void arm(){

  a.write(ESC_MIN);
  b.write(ESC_MIN);
  c.write(ESC_MIN);
  d.write(ESC_MIN);
  
  delay(ESC_ARM_DELAY);

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
  
  // FIVE FUCKING INTERRUPTS !!!
  PCintPort::attachInterrupt(RC_1, rcInterrupt1, CHANGE);
  PCintPort::attachInterrupt(RC_2, rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(RC_3, rcInterrupt3, CHANGE);
  PCintPort::attachInterrupt(RC_4, rcInterrupt4, CHANGE);
  PCintPort::attachInterrupt(RC_5, rcInterrupt5, CHANGE);
  
}

void initMPU(){
  
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
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

void initRegulators(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}

void rcInterrupt1(){
   ch1 = millis() - rcLastChange1;
   rcLastChange1 = millis(); 
}

void rcInterrupt2(){
  ch2 = millis() - rcLastChange2;
  rcLastChange2 = millis();
}

void rcInterrupt3(){
  ch3 = millis() - rcLastChange3;
  rcLastChange3 = millis();
}

void rcInterrupt4(){
  ch4 = millis() - rcLastChange4;
  rcLastChange4 = millis();
}

void rcInterrupt5(){
  ch5 = millis() - rcLastChange5;
  rcLastChange5 = millis();
}

