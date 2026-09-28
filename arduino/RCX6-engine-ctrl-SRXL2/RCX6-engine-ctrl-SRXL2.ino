/* *************************************

************************************* */

#include <Arduino_JSON.h>
#include <strings.h>


#include "RPi_Pico_TimerInterrupt.h"
RPI_PICO_Timer ITimer(0);
#define TIMER_PER_USEC 100


/**************************************************************************************
 * ODOM encoder
 * This needs to be moved to its own library class
 **************************************************************************************/

#include "pio_encoder.h"
#define PIN_SHAFT_ODOM_A 14
#define PIN_SHAFT_ODOM_B 15
PioEncoder shaftEncoder = {PIN_SHAFT_ODOM_A};
#define ODOM_PER_USEC 333 /* 30 Hz odom encoder sample rate, PID */
// OFFSETS to stagger the processing in the interrupt code
#define ODOM_OFFSET_USEC 100
#define PID_OFFSET_USEC 200
#define OMSG_OFFSET_USEC 300

// ODOM POD
#define ODOM_RADUIS_M (0.048/2)
#define ODOM_CIRCUMFERENCE_M (2*PI*ODOM_RADUIS_M)
#define ODOM_ENCODER_COUNT_PER_ROTATION (2000.0)

uint32_t odomTimerLastMs = 0;

int encoderPolarity = 1;
volatile int32_t currEncoderCount[2] = {0, 0}; // [0]: millis timestamp, [1]: encoder count

int32_t lastEncoderCount = 0;
int16_t diffEncoderCount = 0;
// double currTravelMeters = 0;
// double diffTravelMeters = 0;
// double velocityMPS = 0;
uint32_t odomCurrTimeMs = 0;
uint32_t odomLastTimeMs = 0;
uint32_t odomDiffTimeMs = 0;



void ResetEncoderValues() {
  odomCurrTimeMs = millis();
  odomLastTimeMs = odomCurrTimeMs;
  odomDiffTimeMs = 0;

  noInterrupts();
  currEncoderCount[0] = millis(); // timestamp
  currEncoderCount[1] = 0;        // encoder count
  interrupts();

  lastEncoderCount = 0;
  diffEncoderCount = 0;
  // currTravelMeters = 0;
  // diffTravelMeters = 0;
  // velocityMPS = 0;
}

void InitEncoders() {
  shaftEncoder.begin();
  shaftEncoder.reset();

  ResetEncoderValues();
}

// This is called from an interrupt handler - be quick!
// TODO: should it have its own interrupt?
uint32_t odomTickCounter = 0;
uint32_t odomTickCount = 0;
uint32_t pidTickCount = 0;
uint32_t omsgTickCount = 0;
bool sendOdom = false;
bool runPID = false;

// Interrupt handler
void OdomHandler() {

  // odomCurrTimeMs = millis();
  // if(odomCurrTimeMs < (odomLastTimeMs + ODOM_PER_MSEC)) return;

  // Read the drive shaft odom encoder
  if(odomTickCounter > odomTickCount) {
    odomDiffTimeMs = odomCurrTimeMs - odomLastTimeMs;
    odomLastTimeMs = odomCurrTimeMs;
    // read pod encoder
    // noInterrupts();
    currEncoderCount[0] = millis(); // timestamp
    currEncoderCount[1] = shaftEncoder.getCount() * encoderPolarity; // encoder count
    // interrupts();
    // Next odom sample time
    odomTickCount = odomTickCounter + ODOM_PER_USEC;
  }

  // PID code
  if(odomTickCounter > pidTickCount) {
    // // protect stamp,count pair from changing by disabling interrupts
    // noInterrupts();
    float enc = currEncoderCount[1];
    diffEncoderCount = enc - lastEncoderCount;
    lastEncoderCount = enc;
    // interrupts();

    runPID = true;

    // Next PID process time
    pidTickCount = odomTickCounter + ODOM_PER_USEC;
  }
//Serial.print(diffEncoderCount);Serial.print(":");
  // currTravelMeters = (currEncoderCount/ODOM_ENCODER_COUNT_PER_ROTATION)*ODOM_CIRCUMFERENCE_M;
  // diffTravelMeters = (diffEncoderCount/ODOM_ENCODER_COUNT_PER_ROTATION)*ODOM_CIRCUMFERENCE_M;
  // velocityMPS = diffTravelMeters/(1e-6*diffTimeUs);


  // // PID loop to regulate R and L wheel speed based on wheel encoder values
  // for(int enc=0; enc<2; enc++) {
  //   // Limit acceleration and decelleration of wheel velocity
  //   float targetMps = WheelMps[enc];
  //   float limitedMps = WheelMpsAccLimited[enc];
  //   float dMpsAcc = wheelAccellerationMaxMpsPs * (diffTimeUs/1e6);

  //   if(limitedMps != targetMps) {
  //     if(limitedMps < targetMps) {
  //       limitedMps = limitedMps + dMpsAcc;
  //       if(limitedMps > targetMps) limitedMps = targetMps;
  //     } else if(limitedMps > targetMps) {
  //       limitedMps = limitedMps - dMpsAcc;
  //       if(limitedMps < targetMps) limitedMps = targetMps;
  //     } 
  //     WheelMpsAccLimited[enc] = limitedMps;
  //   }

  //   // error is positive when measured velocity > target velocity
  //   PidErrorMps[enc] = velocityMPS[enc] - limitedMps;
  //   PidPropPct[enc] = PidErrorMps[enc] * PidPropCoef[enc];
  //   PidIntPct[enc] += PidErrorMps[enc] * PidIntCoef[enc];
  //   if(PidIntPct[enc] > +PidIntPctMax) PidIntPct[enc] = +PidIntPctMax;
  //   if(PidIntPct[enc] < -PidIntPctMax) PidIntPct[enc] = -PidIntPctMax;

  //   float pct = 100*(PidPropPct[enc] + PidIntPct[enc]);
  //   // TODO: limit pct here? or count on limit check in MotorDrivePct()
  //   MotorPct[enc] = pct;
  // }

  // // Set wheel motor drive PWM percent
  // MotorDrivePct(MotorPct[1], MotorPct[0]);

  // Send odom message to port
  if(odomTickCounter > omsgTickCount) {
    sendOdom = true;
    // Next Odom message time
    omsgTickCount = odomTickCounter + ODOM_PER_USEC;
  }

  odomTickCounter++;
}

/**************************************************************************************
 * 
 **************************************************************************************/


// local files in arduino sketch folder
#include "srxl2.h"
SRXL2 srx;

#include "rcxpwm.h"
RCXPWM pwm;

// TODO: Handle wrap??
// Decode gear shift PWM signal from receiver using pin edge interrupts and timer
void pin_mShift_interrupt (void) {
  pwm.pin_mShift_interruptX();
}
// Decode steering PWM signal from receiver using pin edge interrupts and timer
void pin_mSteer_interrupt (void) {
  pwm.pin_mSteer_interruptX();
}
void attachInterrupts() {
  attachInterrupt(digitalPinToInterrupt(PIN_mSteer), pin_mSteer_interrupt, CHANGE) ;
  attachInterrupt(digitalPinToInterrupt(PIN_mShift), pin_mShift_interrupt, CHANGE) ;
}

/**************************************************************************************
 * GLOBAL VARIABLES
 * Should have suffix "_g"
 **************************************************************************************/

// Kill switch global variables
int killSwLastShift_g = 50000; // down, used for shift action
bool rcvrActive_g = false;
bool killSwEnable_g = false;
bool killSwLatch_g = false;
bool killSwActive_g = false;

bool odomOn = true;

float sThrottlePct = 0.0;
float sSteerPct = 0.0; 
String sShiftGear = "low";

// Loop timer for precise interval periods
unsigned long statusMillis_last = 0;

String mode_g = "bypass"; // power/reset default
int loopStatusPeriod = 1000;

/***************************************
* Hardware PICO TIMER 
*******************************************/
#define TX_ESC_USEC 33333 // 30/sec
// Configure the timer for the SRXL2, used to time TXEN signals
// Also used for Odom wheel encoder
void configTimer() {
  srx.setTimerTickIntervalUsec(TIMER_PER_USEC);
  srx.setTxEscUsec(TX_ESC_USEC); // also syncronizes

  // syncronize the Odom timer
  odomTickCount = odomTickCounter + ODOM_PER_USEC + ODOM_OFFSET_USEC;
  // syncronize the PID loop timer
  pidTickCount = odomTickCounter + ODOM_PER_USEC + PID_OFFSET_USEC;
  // syncronize the odom message timer
  omsgTickCount = odomTickCounter + ODOM_PER_USEC + OMSG_OFFSET_USEC;

  if (ITimer.attachInterruptInterval(TIMER_PER_USEC, timerTick))
    {Serial.print(F("Starting ITimer OK, period usec = ")); Serial.println(TIMER_PER_USEC);}
  else
    {Serial.println(F("Can't set ITimer. Select another freq. or timer"));}
  Serial.flush();  
}

// Tick the timer in SRXL2 and execute Odom Wheel encoder
// Interrupt drive at 10 KHz
bool timerTick(struct repeating_timer *t) { 
  (void) t;
  srx.timerTick();
  OdomHandler();
  watchdogTimer();
  return true;
}

/********************************************************
* Watchdog timer
* Called from timer "Tick" interrupt
*********************************************************/
uint32_t wdTimeStopMsec = 0; // watchdog time in msec, 0=disabled
uint32_t wdTimeMsec = 0;
bool wdTimedOut = true;
// Called from interrupt 
void watchdogTimer() {
  if (wdTimeStopMsec > 0) {
    if (millis() > wdTimeStopMsec) {
      // watchdog timeout
      wdTimedOut = true;
      wdTimeStopMsec = 0; // disable watchdog until new command
      Serial.println("Watchdog timeout, stopping motor");
      resetPID();
      sThrottlePct = 0;
      sSteerPct = 0;
      srx.setThrottlePct(sThrottlePct);
      pwm.setSteerPct(sSteerPct);
    }
  }
}

// Call at each valid command to reset the watchdog timer
void resetWatchdogTimer() {
  if (wdTimeMsec > 0) {
    wdTimeStopMsec = millis() + wdTimeMsec;
  } else {
    wdTimeStopMsec = 0; // disable watchdog
  }
  wdTimedOut = false;
}

/********************************************************
* PID loop code 
* Uses linear M/S from command with the shaft encoder feedback
* And executes the Ackermann conversion after each PID sample
* The angular velocity does not have a PID loop (no servo encoder)
*********************************************************/
float coeffA = 0.2;  // proportional scale
float coeffB = 0.04; // integral scale
float coeffDA = 0.15;  // diff proportional scale
float coeffDB = 0.075; // diff integral scale
float coeffDd = 0.5; // diff decay rate
int encPerMeter = 6000; // encoder counts per meter

float pidInt = 0;
float pidDd = 0; // Diff signal decay
float pidMaxMps = 2.0; // +- maximum meters/sec output and integrator

// TODO: timestamps are uint32_t?
int32_t pidLastStamp = 0;
int32_t pidLastEnc = 0;

// data for the median-3 filter
float medianData[3] = {0,0,0};

// from "cv":[linX,steerRad] command velocity from serial port
float cmdVelLinX = 0;
float cmdVelAngRad = 0;

// Add global variables for last commanded velocities
float lastLinX = 0.0; // wheel velocity in Meter/Sec
float limSteerRad = 0.0; // steering angle in Radians
float lastCmdVelLinX = 0;

void resetPID(){
  runPID = false;
  pidInt = 0;
  pidDd = 0;
  // float enc = currEncoderCount[1]; // stopping interrupts not needed
//Serial.print("resetPID: enc = ");Serial.println(enc);
  pidLastEnc = 0;// enc;
  for(int i=0;i<3;i++) medianData[i] = 0;// enc;
  cmdVelLinX = 0;
  cmdVelAngRad = 0;
  lastCmdVelLinX = 0;
  lastLinX = 0;
  limSteerRad = 0;
  sThrottlePct = 0;
  sSteerPct = 0;
}

void pidLoopCode(){
  if(runPID!=true) return;
  runPID = false;
  if(mode_g!="cv") return;
  if(killSwActive_g) return;

  // PID code
  float linX = 0;
//  float angZ = 0;
  float encMps = 0;
  float err = 0;
  float propErr = 0;
  float intErr = 0;

  int32_t stamp = 0;
  int32_t enc = 0;
  float encTsec = 0;
  float deltaEnc = 0;
  
  float sortM3[3] = {0,0,0};

  // // protect timestamp and encoder value pair from interrupt
  // noInterrupts();
  // stamp = currEncoderCount[0]; // timestamp
  // enc   = currEncoderCount[1]; // encoder count
  // interrupts();


  /****************** start PID *********************/

  // encTsec = (stamp - pidLastStamp)/1000.0; // delta stamp time in msec to seconds
  // pidLastStamp = stamp;
  // if(encTsec>0.1 || encTsec<=0) return; // bad sample
  encTsec = 1e-6*ODOM_PER_USEC*TIMER_PER_USEC; // reduces jitter in time

  // deltaEnc = enc - pidLastEnc;
  // pidLastEnc = enc;
  deltaEnc = diffEncoderCount;
  if(deltaEnc > 15000 || deltaEnc < -15000) deltaEnc=0; // bad sample

  // filter deltaEnc using a median filter - reduce jitter in encoder data
  medianData[1]=medianData[0]; medianData[2]=medianData[1]; // shift in new data
  medianData[0]=deltaEnc;
  for(int i=0;i<3;i++) sortM3[i]=medianData[i];
  if(sortM3[1]<sortM3[0]) std::swap(sortM3[1],sortM3[0]);
  if(sortM3[2]<sortM3[0]) std::swap(sortM3[2],sortM3[0]);
  if(sortM3[2]<sortM3[1]) std::swap(sortM3[2],sortM3[1]);
  float filtEnc = sortM3[1];

  encMps = (filtEnc/encPerMeter)/encTsec; // convert encoder counts to meters/sec

  err = cmdVelLinX - encMps; 
  propErr = err*coeffA;
  intErr  = err*coeffB;

  pidInt += intErr;

  // Add differential signal from change of velocity input
  float diff = cmdVelLinX - lastCmdVelLinX;
  lastCmdVelLinX = cmdVelLinX;
  pidDd += diff - pidDd*coeffDd;
  float propDerr = pidDd*coeffDA;
  float intDerr = pidDd*coeffDB;
  if(fabs(intDerr) < 2.0) pidInt += intDerr;

  // Limit integrator value
  if(pidInt>pidMaxMps) pidInt = pidMaxMps;
  else if(pidInt<-pidMaxMps) pidInt = -pidMaxMps;

  linX = propErr + pidInt + propDerr;
  
  // Limit PID output value
  if(linX>pidMaxMps) linX = pidMaxMps;
  else if(linX<-pidMaxMps) linX = -pidMaxMps;

  /****************** end PID *********************/

// Serial.print("PID");
// Serial.print(", stamp=");
// Serial.print(stamp);
// Serial.print(", encTsec=");
// Serial.print(encTsec, 4);
// Serial.print(", filtEnc=");
// Serial.print(filtEnc);
// Serial.print(", encMps=");
// Serial.print(encMps, 4);
// Serial.print(", err=");
// Serial.print(err, 4);
// Serial.print(", propErr=");
// Serial.print(propErr, 4);
// Serial.print(", intErr=");
// Serial.print(intErr, 4);
// Serial.print(", pidInt=");
// Serial.print(pidInt, 6);
// Serial.print(", linX=");
// Serial.print(linX, 4);
// Serial.println();

  // Ackerman conversion and output to ESC via RXCL2
  //TODO: move output to this function?
  // AckermanConvert(linX, cmdVelAngRad);

  // force linear velocity to 0 when input is zero
  if (cmdVelLinX == 0) linX = 0;
  // saves the resulting percent values in the global variables
  cvToPct(linX, cmdVelAngRad);

  
  if (!wdTimedOut) {
    srx.setThrottlePct(sThrottlePct);
    pwm.setSteerPct(sSteerPct);
  }

}

float throttlePctPerMps = 46.80; // use for both fwd and rvs
float throttlePctDeadZone = 10.0; // this is +- dead zone around 0 percent

// M/S to Percent vs Vbat correction scale factor equation = A*Vbat + B
const float throttlePctVbatScaleA = 0.078545; // from test deltaEnc vs Vbat on bench 
const float throttlePctVbatScaleB = 0.018182; // from test deltaEnc vs Vbat on bench

// Ackermann steering parameters
const float wheelBase = 0.490; // meters, front to back
const float trackWidth = 0.310; // meters, left to right
// const float maxSteeringRad = 0.523; // ~30 deg, adjust for your hardware
//const float maxSteeringRad = 0.611; // ~35 deg at steer percent=100
const float maxSteeringRad = 0.698; // ~40 deg at steer percent=100

//float steeringAngleRad = 0.0;

float throttlePctPerMpsScale = 1.0;  // mpy 1.0 = no scale
// float throttlePctDeadZoneAdj = 0.0;  // add Adj
float steerCenterPctAdj = 0.0;       // add Adj
float wheelBaseAdj = 0.0;            // add Adj

// TODO: read values in PID loop and calculate using scale in PID loop?
// currently these are read in the command process loop
float escVin = 12.0; // default value
int motorRpm = 0; // default value


/********************************************************
* Convert linear velocity and steering angle to percent
* linX Meters/Sec
* angle in Radians
* updates sThrottlePct and sSteerPct
*********************************************************/
void cvToPct(float linX, float angle) {
  float scale = (throttlePctVbatScaleA * escVin) + throttlePctVbatScaleB;
  float dz = throttlePctDeadZone;
  float ppm = throttlePctPerMps * throttlePctPerMpsScale; // scale percent to meters
  float cmdVelAngRad = angle;

  if(linX==0.0) sThrottlePct = 0;
  else if(linX>0.0) {
    sThrottlePct = (dz-0) + (linX * ppm);
    sThrottlePct /= scale; // apply Vbat correction
    if(sThrottlePct > 100) sThrottlePct = 100;
    if(sThrottlePct < dz) sThrottlePct = 0;
    // // convert limited pct back to linear velocity x m/s
    // linX = (sThrottlePct - (dz-1)) / ppm;
    // linX *= scale; // apply reverse Vbat correction
  } else {
    sThrottlePct = -(dz-0) + (linX * ppm);
    sThrottlePct /= scale; // apply Vbat correction
    if(sThrottlePct < -100) sThrottlePct = -100;
    if(sThrottlePct > -dz) sThrottlePct = 0;
    // // convert limited pct back to linear velocity x m/s
    // linX = (sThrottlePct + (dz-1)) / ppm;
    // linX *= scale; // apply reverse Vbat correction
  }

  // Convert steering angle to percent
  sSteerPct = (100.0 * (cmdVelAngRad / maxSteeringRad)) + steerCenterPctAdj;
  if(sSteerPct > 100) sSteerPct = 100;
  if(sSteerPct < -100) sSteerPct = -100;

   // calc limited steer angle
  cmdVelAngRad = (sSteerPct-steerCenterPctAdj)/100.0 * maxSteeringRad;

  
  lastLinX = linX;
  limSteerRad = cmdVelAngRad;
}


/********************************************************
* USB serial JSON code
*********************************************************/
// retrieve JSON messages from USB serial port
void getJsonMsgs() {
  if (Serial.available()== 0) return;

  // read the incoming string and parse it
  String incomingString = Serial.readStringUntil('\n');
  jsonParse(incomingString.c_str());

}

bool jsonParse(const char *jsonStr) {
  //Serial.println(jsonStr);

  JSONVar myObject = JSON.parse(jsonStr);

//  if (JSON.typeof(myObject) == "undefined") {
  if (JSON.typeof(myObject) != "object") {
    Serial.println("Parsing JSON string input failed! Not a valid object");
    return false;
  }

  resetWatchdogTimer();

  // mode bypass/passthru/cv/pct
  // bypass and passthru use the RC transmitter to control
  // cv and pct use the serial commands:
  //  cv for command velocity style "cv":[linX, angZ] radians/sec float
  //  pct for percent control "thr":N and "str":N percent float
  if (myObject.hasOwnProperty("mode")) {
    mode_g = (String) myObject["mode"];
    Serial.print("mode = ");
    Serial.println(mode_g);
    
    configureMode(); //TODO: move to srx.setMode
    srx.setMode(mode_g);
    pwm.setMode(mode_g);
  }

  // Watchdog time integer milli-seconds, 0=disabled
  // any command will reset the watchdog timer
  // If watchdog times out then the motor stops
  if (myObject.hasOwnProperty("wd")) {
    wdTimeMsec = (int) myObject["wd"];
    // Serial.print("wdTimeMsec = ");
    // Serial.println(wdTimeMsec);
    resetWatchdogTimer();
  }

  if(mode_g == "cv") {
    // Command Velocity array [linear_X, steer_rad]
    if (myObject.hasOwnProperty("cv")) {
      JSONVar cv;
      cv = myObject["cv"];
      // Serial.print("CmdVar = [linx=");
      // Serial.print((double)cv[0]);
      // Serial.print(", steer=");
      // Serial.print((double)cv[1]);
      // Serial.println("]");

      //resetPID();

      cmdVelLinX = (double)cv[0];
      cmdVelAngRad = (double)cv[1];
    }

    // PID loop coefficients [prop, int, diffProp, diffInt]
    if (myObject.hasOwnProperty("pid")) {
      JSONVar pid;
      pid = myObject["pid"];
      coeffA = (double)pid[0];
      coeffB = (double)pid[1];
      coeffDA = (double)pid[2];
      coeffDB = (double)pid[3];

      Serial.print("pid = [coeffA=");
      Serial.print(coeffA);
      Serial.print(", coeffB=");
      Serial.print(coeffB);
      Serial.print(", [coeffDA=");
      Serial.print(coeffDA);
      Serial.print(", coeffDB=");
      Serial.print(coeffDB);
      Serial.println("]");
    }

  } else {
    cmdVelLinX = 0;
    cmdVelAngRad = 0;
  }

  if (myObject.hasOwnProperty("odom")) {
    odomOn = (bool)myObject["odom"];
    if (odomOn)
      Serial.println("odom messages are ON");
    else
      Serial.println("odom messages are OFF");
  }

  if (mode_g == "pct") {
    // Steering percent
    if (myObject.hasOwnProperty("str")) {
      sSteerPct = (double) myObject["str"];
      Serial.print("steer = ");
      Serial.println(sSteerPct);
      // add steering percent offset
      pwm.setSteerPct(sSteerPct);
      srx.setSteerPct(sSteerPct);
    }
    // Throttle percent
    if (myObject.hasOwnProperty("thr")) {
      sThrottlePct = (double) myObject["thr"];
      Serial.print("throttle = ");
      Serial.println(sThrottlePct);
      srx.setThrottlePct(sThrottlePct);
    }
  } 
  else {
    sThrottlePct = 0;
    sSteerPct = 0;
  }


  // Shift gear high/low
  if (myObject.hasOwnProperty("sft")) {
    sShiftGear = (String) myObject["sft"];
    Serial.print("gear = ");
    Serial.println(sShiftGear);
    pwm.setShiftGear(sShiftGear=="high"?"high":"low");
    srx.setShiftGear(sShiftGear=="high"?"high":"low");
  }

  // Throttle trim - floats - 0 is no trim for all
  // [
  // throttlePctPerMpsScale,  mpy scale 
  // throttlePctDeadZone,  
  // steerCenterPctAdj,       add Adj
  // wheelBaseAdj             add Adj
  //]
  if (myObject.hasOwnProperty("trim")) {
    JSONVar trim;
    trim = myObject["trim"];
    throttlePctPerMpsScale = 1+(double)trim[0]; // trim[0]==0 no trim
    throttlePctDeadZone = (double)trim[1];
    steerCenterPctAdj      = (double)trim[2];
    wheelBaseAdj           = (double)trim[3];
    Serial.print("throttlePctPerMpsScale = ");
    Serial.print(throttlePctPerMpsScale, 3);
    Serial.print(", throttlePctDeadZone = ");
    Serial.print(throttlePctDeadZone, 3);
    Serial.print(", steerCenterPctAdj = ");
    Serial.print(steerCenterPctAdj, 3);
    Serial.print(", wheelBaseAdj = ");
    Serial.println(wheelBaseAdj, 3);
  }


//  // get Battery Voltage and RPM from telemetry
//  // move to PID loop?
//  escVin = srx.getEscVin();
//  motorRpm = srx.getEscRpm();

  return true;
}

//TODO: move to SRXL2
void configureMode(){
  if(mode_g == "bypass") {
    digitalWrite(BYPASS, 0); // enable bypass switch 
  }
  else if(mode_g == "passthru" || mode_g == "cv" || mode_g == "pct") {
    digitalWrite(BYPASS, 1); // disable bypass switch
  }

  // to be safe reset PID when mode changes
  resetPID();

  // to be safe disable SRXL2 serial transmits
  srx.disableTX();
}

// USB Serial
void configSerial(){
  Serial.begin(1000000);
  while(!Serial) delay(100);
  Serial.println("USB Serial started");
}


/*
Engine Controller Firmware (RCX6-engine-ctrl-SRXL2)
Controls engine, steering, and shift functions for the RCX6 robot using SRXL2 serial protocol.
Handles multiplexing between RC receiver and computer control, relays commands, and manages telemetry.

Main Firmware (RCX6-engine-ctrl-SRXL2.ino):
- Initializes hardware, sets up serial communication, and manages the main control loop.
- Handles switching between RC and computer control modes.
- Processes incoming JSON commands and relays them to the appropriate subsystems.

PWM Control (rcxpwm.cpp, rcxpwm.h):
- Implements PWM signal generation and decoding for motor, steering, and shift channels.
- Provides functions to read and write PWM values for both RC and computer control.
- Manages safe switching and signal integrity.

SRXL2 Protocol (srxl2.cpp, srxl2.h, srxl2Structs.h):
- Implements the SRXL2 serial protocol for communication with Spektrum ESC and telemetry devices.
- Defines SRXL2 message structures and parsing logic.
- Handles bidirectional UART communication, message encoding/decoding, and telemetry extraction.

Libraries Used:
- Arduino core libraries
- SRXL2 protocol library (custom implementation)
- Standard C++ libraries for data structures and serial communication
*/

// This is an interrupt handler
// send the odometry message to serial port
// this is used by ROS2 for its odom message
// {"odom": {"stamp":T, "enc":E, "linx":X, "steer":R}}
void sendOdomMsg() {
  JSONVar data;
  JSONVar odom;
  uint32_t stamp, enc;

  // this routine is part of the interrupt handler
  // noInterrupts();
  stamp = currEncoderCount[0]; // timestamp
  enc = currEncoderCount[1]; // encoder count
  // interrupts();

  data["stamp"] = (uint32_t)stamp;
  data["enc"] = (int32_t)enc; // encoder values are + or -

  data["linx"] = (float)lastLinX;
  data["steer"] = (float)limSteerRad;

  odom["odom"] = data;

  String jsonString = JSON.stringify(odom);
  Serial.println(jsonString);
}

// Send status message to host computer
void sendStatusMsg() {

  // get Battery Voltage and RPM from telemetry
  // move to PID loop?
  escVin = srx.getEscVin();
  motorRpm = srx.getEscRpm();

  JSONVar statuses;

  statuses["mode"] = mode_g;
  statuses["vbat"] = escVin;
  statuses["rca"] = rcvrActive_g;
  statuses["kse"] = killSwEnable_g;
  statuses["ksl"] = killSwLatch_g;
  statuses["ksa"] = killSwActive_g;

  JSONVar status;
  status["status"] = statuses;

  String jsonString = JSON.stringify(status);
  Serial.println(jsonString);

}


// Decode receiver signal for transmitter active
uint32_t rcvDecodeMillis_last_g = 0;
int rcvDecodePeriod_g = 20; // 50/sec

void killSwDecode(void);
void killSwReset(void);

void rcvDecode(uint32_t loopMillis) {  
  if ((loopMillis - rcvDecodeMillis_last_g) < rcvDecodePeriod_g) return;
  rcvDecodeMillis_last_g = loopMillis;

  bool rcTxActive = srx.getRcTransmitterActive();
//Serial.printf("thr %d, str %d, sft %d rcTxActive %d\n", throttle, steer, shift, rcTxActive);


  // Detect if the receiver is detecting signals from transmitter
  if(rcTxActive) {
    if(!rcvrActive_g) { // execute once
      rcvrActive_g = true;
      Serial.println("Receiver is now active");
    }
    killSwDecode(); // execute continuously not once
  }
  else {
    if(rcvrActive_g) { // execute once
      rcvrActive_g = false;
      Serial.println("Receiver is now NOT active");
      killSwReset();
    }
  }
}

String killSwLastMode_g = "bypass";
int killCnt_g = 0;

void killSwReset(void) {
  killSwLastShift_g = 50000; // down
  killSwEnable_g = false;
  killSwActive_g = false;
  killSwLatch_g = false;
  killSwLastMode_g = mode_g;
}

// Decode receiever steering and shift and throttle
// Center = 32,768 (2^16)
// The shift action <20,000(up) the >40,000(dn = low)

#define killSwHiLvl 40000
#define killSwLoLvl 20000

bool ksl0_g = false;

void killSwDecode(void) {

  int throttle = srx.getRcvThrottle();
  int steer    = srx.getRcvSteer();
  int shift    = srx.getRcvShift();

  bool throttleFWD = throttle>killSwHiLvl;
  bool throttleREV = throttle<killSwLoLvl;
  bool throttleZER = !throttleFWD && !throttleREV;
  bool steerCW     = steer>killSwHiLvl;
  bool steerCCW    = steer<killSwLoLvl;
  bool shiftAction = false;
  bool modeChange  = false;

    // filter throttle since no shift action
  if(throttleFWD && (killCnt_g<10)) killCnt_g++;
  if(throttleZER && (killCnt_g>0))  killCnt_g--;

  // Detect shift action switch is up then down, only occurs once per shift sequence
  // TODO: does this need a filter?
  if(killSwLastShift_g < killSwLoLvl) { // last shift sw position was UP (high gear)
    if(shift > killSwHiLvl) { // current shift sw position is DOWN (low gear)
      shiftAction = true;
    }
  }
  killSwLastShift_g = shift;

  // Detect mode change using receiver signals
  if (shiftAction && throttleZER && steerCW) {
    mode_g = "cv";
  }
  else if(shiftAction && throttleZER && steerCCW) {
    mode_g = "bypass";
  }
  if(mode_g != killSwLastMode_g) { // execute once
    modeChange = true;
    killSwReset();
    configureMode();
    srx.setMode(mode_g);
    pwm.setMode(mode_g);
  }
  killSwLastMode_g = mode_g;

  // Only run the the kill switch code when in "cv" mode
  if(mode_g != "cv") {
    return;
  }

  // decode kill switch enable, inactive if rcvr tx inactive or reset
  if(modeChange) {
    killSwEnable_g = false;
  }
  else if(shiftAction && throttleFWD) {
    killSwEnable_g = true;
  }

  // decode kill switch active, no shift action
  // The throttle signal is not stable and needs to be filtered
  if (!killSwEnable_g) {
    killCnt_g = 0;
    if(killSwActive_g) { // one shot
      killSwActive_g = false;
      sendKillSwActiveStatus(false);
    }    
  } 
  else {
    // // filter throttle since no shift action
    // if(throttleFWD && (killCnt_g<10)) killCnt_g++;
    // if(throttleZER && (killCnt_g>0))  killCnt_g--;

    if(killSwActive_g && (killCnt_g>=10)) { // one shot
      killSwActive_g = false;
      sendKillSwActiveStatus(false);
    }
    if(!killSwActive_g && (killCnt_g<=0) && !killSwLatch_g) { // one shot
      killSwActive_g = true;
      resetPID();
      sThrottlePct = 0;
      sSteerPct = 0;
      srx.setThrottlePct(sThrottlePct);
      pwm.setSteerPct(sSteerPct);
      sendKillSwActiveStatus(true);
    }
  }

  // decode kill switch latch, shift action toggle
  if (!killSwEnable_g || (killSwLatch_g && killCnt_g>=10)) {
    killSwLatch_g = false;
    ksl0_g = false;
  }
  else if(!killSwLatch_g && !killSwActive_g && shiftAction) {
    // Latch when throttle is activated and shift up/dn
    ksl0_g = true;
  }
  else if (ksl0_g && killCnt_g<=5) {
    // Latch after throttle is released
    killSwLatch_g = true;
    ksl0_g = false;
  }

  // Debug print
  if(shiftAction) {
    Serial.printf("shiftAction: thr=%d str=%d sft=%d mode=%s kse=%d ksl=%d ksa=%d\n", 
        throttle, steer, shift, mode_g.c_str(), killSwEnable_g, killSwLatch_g, killSwActive_g);
  }

}

void sendKillSwActiveStatus(bool ksa) {
  JSONVar killStatus;
  killStatus["ksa"] = ksa;
  JSONVar status;
  status["status"] = killStatus;
  String jsonString = JSON.stringify(status);
  Serial.println(jsonString);
}

void pwmLoopCode() {
  pwm.loopCode();
}

void srxLoopCode() {
  srx.loopCode();
}

// Send messages to serial port (to ROS node)
// Called from loop code
void sendMsgs(unsigned long loopMillis) {
  // send status message 1/sec (every loopStatusPeriod msec)
  if ((loopMillis - statusMillis_last) >= loopStatusPeriod) {
    sendStatusMsg();
    statusMillis_last = loopMillis;
  }

  // send Odom wheel encoder data
  if (odomOn && sendOdom) {
    sendOdom = false;
    sendOdomMsg();
  }
}


/****************************************************/
void setup() {
  srx.configPins();
  configSerial();
  srx.configSerialESC();
  srx.configSerialRCV();
  configTimer();
  pwm.configPWM();
  attachInterrupts();
  InitEncoders();

}

/****************************************************/
void loop() {
  unsigned long loopMillis = millis();

  // impliment failsafe mechanism
  // checkFailsafe();

  // Decode selections from receiver signals
  rcvDecode(loopMillis);

  getJsonMsgs();

  srxLoopCode();

  pwmLoopCode();

  pidLoopCode();

  sendMsgs(loopMillis);

}
