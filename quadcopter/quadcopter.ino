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
#include <RunningMedian.h>

//#define DEBUG_BAL
//#define DEBUG_RC
//#define DEBUG_YPR
#define DEBUG_MOTORS
#define DEBUG


// Arduino Pin configuration
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


// ESC configuration
#define ESC_MIN 22
#define ESC_MAX 115
#define ESC_OFF 0
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 5000


// RC configuration
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
#define RC_TIMEOUT 200000
#define RC_ERROR_BOUND 50
#define RC_MIN_CHANGE_MODE 1250
#define RC_MAX_CHANGE_MODE 1850


// PID configuration
#define PITCH_P_VAL 1
#define PITCH_I_VAL 0.0
#define PITCH_D_VAL 0.5

#define ROLL_P_VAL 1
#define ROLL_I_VAL 0.0
#define ROLL_D_VAL 0.5

#define YAW_P_VAL 0.5
#define YAW_I_VAL 0
#define YAW_D_VAL 0.5


// Flight parameters
#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20
#define YPR_ROUNDING_BASE 0.04
#define YPR_NUMBER_SAMPLES 1
#define YPR_SAMPLES_MEDIAN 1
#define YPR_MAX_IGNORE_ERRORS 3
#define YPR_MAX_CHANGE 0.28


// Quadcopter modes
#define START_MODE 0                   // mode -> not initialized
#define INIT_MODE  1                   // mode -> initialized, no flying started
#define FLY_MODE   2                   // mode -> flying
#define LAND_MODE  3                   // mode -> RC error, slowly decrease ESCs power
#define STOP_MODE  4                   // mode -> emergency stop, ESCs setted to ESC_OFF


// MPU variables
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
int yprErrorCount[3] = {0, 0, 0};

volatile bool mpuInterrupt = false;    //interrupt flag


// Interrupt lock
boolean interruptLock = false;

// RC variables
float ch1, ch2, ch3, ch4, ch5;         // RC channel inputs
unsigned int rc_errors = 0;

// Motor controll variables
int velocity;                          // global velocity
float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd
int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;                        // velocity of axes
Servo a,b,c,d;

// PID variables
PID yawReg(&ypr[0], &bal_axes, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);
PID pitchReg(&ypr[1], &bal_bd, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &bal_ac, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);

// Filter variables
float ch1Last, ch2Last, ch4Last, velocityLast;
RunningMedian yawMedian = RunningMedian(YPR_NUMBER_SAMPLES);
RunningMedian pitchMedian = RunningMedian(YPR_NUMBER_SAMPLES);
RunningMedian rollMedian = RunningMedian(YPR_NUMBER_SAMPLES);


// Quadcopter status
int mode = START_MODE;                  


// SETUP function
void setup(){
  
  initRC();
  initMPU();
  initESCs();
  initBalancing();
  initRegulators();
  mode = INIT_MODE;                   // Change mode
  
  #ifdef DEBUG                        // Device tests go here
    Serial.begin(9600);               // Serial only necessary if in DEBUG mode
    Serial.flush();
    Serial.println("END OF SETUP FUNCTION");
  #endif
}

// LOOP function
void loop(){
  switch(mode) {
    case START_MODE:
      setup();
      break;
    case INIT_MODE:
      getRC();
      break;
    case FLY_MODE:
      flyingMode();
      break;
    case LAND_MODE:
      landingMode();
      break;
    case STOP_MODE:
      stopMode();
      break;
    default:
      mode = START_MODE;
      break;
  }  
}

// Performs the loop acctions for the flying mode(2)
void flyingMode() {
  while(!mpuInterrupt && fifoCount < packetSize){     
    /* Do nothing while MPU is not working
     * This should be a VERY short period
     */      
  }
  
  getRC();
  getYPR();                          
  computePID();
  calculateVelocities();
  updateMotors();
}

// Performs the loop acctions for the landing mode(3)
void landingMode() {
  mode = INIT_MODE;
}

// Performs the loop acctions for the landing mode(4)
void stopMode() {
  va = ESC_OFF;
  vb = ESC_OFF;
  vc = ESC_OFF;
  vd = ESC_OFF;
  updateMotors();                          // Stop motors
  
  delay(20);
  mode = START_MODE;                       // Restart quadcopter
}


// Checks two times if RC is in start fly position. If it's, changes the mode
void setFlyMode() {
  delay(10);
  ch1 = pulseIn(RC_1, HIGH, RC_TIMEOUT);
  ch2 = pulseIn(RC_2, HIGH, RC_TIMEOUT);
  ch3 = pulseIn(RC_3, HIGH, RC_TIMEOUT);
  ch4 = pulseIn(RC_4, HIGH, RC_TIMEOUT);
  if (ch1 > RC_MAX_CHANGE_MODE && ch2 < RC_MIN_CHANGE_MODE && ch3 < RC_MIN_CHANGE_MODE && ch4 < RC_MIN_CHANGE_MODE) {
    delay(20);
    ch1 = pulseIn(RC_1, HIGH, RC_TIMEOUT);
    ch2 = pulseIn(RC_2, HIGH, RC_TIMEOUT);
    ch3 = pulseIn(RC_3, HIGH, RC_TIMEOUT);
    ch4 = pulseIn(RC_4, HIGH, RC_TIMEOUT);
    if (ch1 > RC_MAX_CHANGE_MODE && ch2 < RC_MIN_CHANGE_MODE && ch3 < RC_MIN_CHANGE_MODE && ch4 < RC_MIN_CHANGE_MODE) {
      mode = FLY_MODE;
    }
  }
}


// Checks two times if RC is in stop position. If it's, changes the mode
void setStopMode() {
  delay(10);
  ch1 = pulseIn(RC_1, HIGH, RC_TIMEOUT);
  ch2 = pulseIn(RC_2, HIGH, RC_TIMEOUT);
  ch3 = pulseIn(RC_3, HIGH, RC_TIMEOUT);
  ch4 = pulseIn(RC_4, HIGH, RC_TIMEOUT);
  if (ch1 < RC_MIN_CHANGE_MODE && ch2 < RC_MIN_CHANGE_MODE && ch3 < RC_MIN_CHANGE_MODE && ch4 > RC_MAX_CHANGE_MODE) {
    delay(20);
    ch1 = pulseIn(RC_1, HIGH, RC_TIMEOUT);
    ch2 = pulseIn(RC_2, HIGH, RC_TIMEOUT);
    ch3 = pulseIn(RC_3, HIGH, RC_TIMEOUT);
    ch4 = pulseIn(RC_4, HIGH, RC_TIMEOUT);
    if (ch1 < RC_MIN_CHANGE_MODE && ch2 < RC_MIN_CHANGE_MODE && ch3 < RC_MIN_CHANGE_MODE && ch4 > RC_MAX_CHANGE_MODE) {
      mode = STOP_MODE;
    }
  }
}


// Read new values from reciver
void getRC() {
  ch1 = pulseIn(RC_1, HIGH, RC_TIMEOUT);
  ch2 = pulseIn(RC_2, HIGH, RC_TIMEOUT);
  ch3 = pulseIn(RC_3, HIGH, RC_TIMEOUT);
  ch4 = pulseIn(RC_4, HIGH, RC_TIMEOUT);
  ch5 = pulseIn(RC_5, HIGH, RC_TIMEOUT);
  rc_errors += (ch1 == 0 && ch2 == 0 && ch3 == 0 && ch4 == 0 && ch5 == 0) ? 1 : -rc_errors;
  
  if (rc_errors > RC_ERROR_BOUND) {
    #ifdef DEBUG_RC
      Serial.println("getRC\tIn Error Mode");
    #endif
    //setErrorMode();
  }
  
  if (mode != FLY_MODE && ch1 > RC_MAX_CHANGE_MODE && ch2 < RC_MIN_CHANGE_MODE && ch3 < RC_MIN_CHANGE_MODE && ch4 < RC_MIN_CHANGE_MODE) {
    setFlyMode();
  }
  else if (ch1 < RC_MIN_CHANGE_MODE && ch2 < RC_MIN_CHANGE_MODE && ch3 < RC_MIN_CHANGE_MODE && ch4 > RC_MAX_CHANGE_MODE) {
    setStopMode();
  }
}


/*  computePID function
 *
 *  formats data for use with PIDLib
 *  and computes PID output
 */

void computePID(){
  
  ch1 = floor(ch1/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch2 = floor(ch2/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch4 = floor(ch4/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;

  ch2 = map(ch2, RC_LOW_CH2, RC_HIGH_CH2, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH4, RC_HIGH_CH4, YAW_MIN, YAW_MAX);
  
  if((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;
  
  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;
  
  ypr[0] = ypr[0] * 180/M_PI;
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;
  
#ifdef DEBUG_RC
  printRC();
#endif
  
  rollReg.Compute();
  pitchReg.Compute();
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
    int i;
    
    if(mpuIntStatus & 0x02){
    
      while (fifoCount < packetSize * YPR_NUMBER_SAMPLES) fifoCount = mpu.getFIFOCount();
      
      yawMedian.clear();
      pitchMedian.clear();
      rollMedian.clear();
  
      for (i = 0; i < YPR_NUMBER_SAMPLES; ++i) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);      
        fifoCount -= packetSize;    
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        float tmp_ypr[3];
        mpu.dmpGetYawPitchRoll(tmp_ypr, &q, &gravity);
        yawMedian.add(tmp_ypr[0]);
        pitchMedian.add(tmp_ypr[1]);
        rollMedian.add(tmp_ypr[2]);
      }
    
      ypr[0] = yawMedian.getAverage(YPR_SAMPLES_MEDIAN);
      ypr[0] = floor(ypr[0]/YPR_ROUNDING_BASE)*YPR_ROUNDING_BASE;
      ypr[1] = pitchMedian.getAverage(YPR_SAMPLES_MEDIAN);
      ypr[1] = floor(ypr[1]/YPR_ROUNDING_BASE)*YPR_ROUNDING_BASE;
      ypr[2] = rollMedian.getAverage(YPR_SAMPLES_MEDIAN);
      ypr[2] = floor(ypr[2]/YPR_ROUNDING_BASE)*YPR_ROUNDING_BASE;
    }
    
    if((mpuIntStatus & 0x10) || fifoCount > 1024){ 
      mpu.resetFIFO();
    }
    
    if ((ypr[0] > 1.5 || ypr[0] < -1.5 || abs(ypr[0] - yprLast[0]) > YPR_MAX_CHANGE) && yprErrorCount[0] < YPR_MAX_IGNORE_ERRORS) {
      ypr[0] = 0.0;
      ++yprErrorCount[0];
    } else {
      float diff = ypr[0] - yprLast[0];
      yprLast[0] = ypr[0];
      ypr[0] = diff;
      yprErrorCount[0] = 0;
    }
    
    if ((ypr[1] > 1.5 || ypr[1] < -1.5 || abs(ypr[1] - yprLast[1]) > YPR_MAX_CHANGE) && yprErrorCount[1] < YPR_MAX_IGNORE_ERRORS) {
      ypr[1] = yprLast[1];
      ++yprErrorCount[1];
    } else {
      yprLast[1] = ypr[1];
      yprErrorCount[1] = 0;
    }
    
    if ((ypr[2] > 1.5 || ypr[2] < -1.5 || abs(ypr[2] - yprLast[2]) > YPR_MAX_CHANGE) && yprErrorCount[2] < YPR_MAX_IGNORE_ERRORS) {
      ypr[2] = yprLast[2];
      ++yprErrorCount[2];
    } else {
      yprLast[2] = ypr[2];
      yprErrorCount[2] = 0;
    }
}

void printYPR() {
  Serial.print("YPR\tyaw ");
  Serial.print(ypr[0]);
  Serial.print("\tpitch ");
  Serial.print(ypr[1]);
  Serial.print("\troll ");
  Serial.println(ypr[2]);
}


void printRC() {
  Serial.print("getRC\tch1=");
    Serial.print(ch1);
    Serial.print("\tch2=");
    Serial.print(ch2);
    Serial.print("\tch3=");
    Serial.println(ch3);
    Serial.print("getRC\tch4=");
    Serial.print(ch4);
    Serial.print("\tch5=");
    Serial.println(ch5);
    Serial.print("getRC\trc_errors=");
    Serial.println(rc_errors);
}

void printBals() {
  Serial.print("BALS\taxes ");
  Serial.print(bal_axes);
  Serial.print("\tb-d ");
  Serial.print(bal_bd);
  Serial.print("\ta-c ");
  Serial.println(bal_ac);
}



/*  calculateVelocities function
 *  
 *  calculates the velocities of every motor
 *  using the PID output
 */

void calculateVelocities(){
  
#ifdef DEBUG_YPR
  printYPR();
#endif
#ifdef DEBUG_BAL
  printBals();
#endif

  ch3 = floor(ch3/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  velocity = map(ch3, RC_LOW_CH3, RC_HIGH_CH3, ESC_MIN, ESC_MAX);

  if((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;
  
  velocityLast = velocity;
  
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
  
}

void updateMotors() {

  a.write(va);
  c.write(vc);
  b.write(vb);
  d.write(vd);

  #ifdef DEBUG_MOTORS
    Serial.print("MOTORS\tva=");
    Serial.print(va);
    Serial.print("\tvb=");
    Serial.print(vb);
    Serial.print("\tvc=");
    Serial.print(vc);
    Serial.print("\tvd=");
    Serial.println(vd);
  #endif
}

void arm(){

  a.write(ESC_MIN);
  b.write(ESC_MIN);
  c.write(ESC_MIN);
  d.write(ESC_MIN);
  
  delay(ESC_ARM_DELAY);

}

void dmpDataReady() {
    mpuInterrupt = true;
}

void initRC(){
  pinMode(RC_1, INPUT);
  pinMode(RC_2, INPUT);
  pinMode(RC_3, INPUT);
  pinMode(RC_4, INPUT);
  pinMode(RC_5, INPUT);
  
  pinMode(RC_PWR, OUTPUT);
  digitalWrite(RC_PWR, HIGH);
  rc_errors = 0;
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

#endif


