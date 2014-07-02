#ifndef QUADARDU
#define QUADARDU

#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

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
#define RC_3 8
#define RC_4 7
#define RC_5 4
#define RC_PWR 10
#define RC_TIMEOUT 200000
#define RC_ERROR_BOUND 50


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

#define RC_HIGH_CH1 2000
#define RC_LOW_CH1 1000
#define RC_HIGH_CH2 2000
#define RC_LOW_CH2 1000
#define RC_HIGH_CH3 2000
#define RC_LOW_CH3 1000
#define RC_HIGH_CH4 2000
#define RC_LOW_CH4 1000
#define RC_HIGH_CH5 2000
#define RC_LOW_CH5 1000
#define RC_ROUNDING_BASE 50

/*  PID configuration
 *  
 */

#define PITCH_P_VAL 0.5
#define PITCH_I_VAL 0
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

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20


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
float yprLast[3] = {0.0f, 0.0f, 0.0f};

volatile bool mpuInterrupt = false;    //interrupt flag

/* Interrupt lock
 *
 */
 
boolean interruptLock = false;

/*  RC variables
 *
 */

float ch1, ch2, ch3, ch4, ch5;         // RC channel inputs
unsigned int rc_errors = 0;

/*  Motor controll variables
 *
 */

int velocity;                          // global velocity

float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd

int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;                        // velocity of axes

Servo a,b,c,d;


/*  Filter variables
 *
 */
 
float ch1Last, ch2Last, ch4Last, velocityLast;

/*  Setup function
 *
 */

void setup(){
  
  initMPU();
  
  #ifdef DEBUG                        // Device tests go here
  
  Serial.begin(9600);                 // Serial only necessary if in DEBUG mode
  Serial.flush();
  
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
     Serial.println(" * ");
      
  }
  
  getYPR();   
  
  Serial.print("PID\tyaw\t");
  Serial.print(ypr[0]);
  Serial.print("\tpitch\t");
  Serial.print(ypr[1]);
  Serial.print("\troll\t");
  Serial.println(ypr[2]);
  
}

void getYPR(){
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024){ 
      
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


void dmpDataReady() {
    mpuInterrupt = true;
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

#endif


