/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
November  2013     V2.3
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "Output.h"
#include "RX.h"
#include "Sensors.h"
#include "Serial.h"
#include "Protocol.h"

#include <avr/pgmspace.h>

#include <math.h>

#include "motors.h"


uint16_t heightMeas = 0;//unknown units
uint16_t lastHeightMeas = 0;
uint16_t heightMeasCounter = 0;
uint16_t lastHeightMeasCounter = 0;
uint16_t lastHeightMeasTime_us = 0;//
float estHeight = 0; //unknown units
float estVelocity = 0;//unknown units per second
int16_t heightMeasDiff  = 0;

//mwm: Here we decide which vehicle we want to use
// They have different controller gains, and the quad spinner does not allow
// you to change the yaw with the joystick. The mapping to the individual
// motors is also different.

#define SPINNER_TYPE_TRI
//#define SPINNER_TYPE_QUAD

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max
/*********** RC alias *****************/

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
  #if ACC
    "ANGLE;"
    "HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    "BARO;"
  #endif
  #if MAG
    "MAG;"
    "HEADFREE;"
    "HEADADJ;"
  #endif
;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
  #if ACC
    1, //"ANGLE;"
    2, //"HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    3, //"BARO;"
  #endif
  #if MAG
    5, //"MAG;"
    6, //"HEADFREE;"
    7, //"HEADADJ;"
  #endif
};

//mwm estimation stuff
float estYawAngle = 0;
float normalx = 0;//est tilt, x component
float normaly = 0;//est tilt, y component
float normalz = 1;//est tilt, z component

uint32_t currentTime_us = 0;
uint16_t previousTime_us = 0;
uint16_t cycleTime_us = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
uint16_t calibratingG;
int16_t  magHold;
uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
uint8_t  rcOptions[CHECKBOXITEMS];
int16_t  sonarAlt;
int16_t  errorAltitudeI = 0;

// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = {0,0,0};

imu_t imu;

analog_t analog;

alt_t alt;

att_t att;

#if defined(ARMEDTIMEWARNING)
  uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

int16_t  debug[4];

flags_struct_t f;

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  uint16_t cycleTimeMax = 0;       // highest ever cycle timen
  uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
  int32_t  BAROaltMax;             // maximum value
  uint16_t GPS_speedMax = 0;       // maximum speed from gps
  uint16_t powerValueMaxMAH = 0;
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
  uint32_t armedTime = 0;
#endif

int16_t  i2c_errors_count = 0;
int16_t  annex650_overrun_count = 0;



#if defined(THROTTLE_ANGLE_CORRECTION)
  int16_t throttleAngleCorrection = 0;	// correction of throttle in lateral wind,
  int8_t  cosZ = 100;					// cos(angleZ)*100
#endif



// **********************
// power meter
// **********************
#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
  uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
  uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
  uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
  uint16_t powerValue = 0;          // last known current
#endif
uint16_t intPowerTrigger1;

// **********************
// telemetry
// **********************
#if defined(LCD_TELEMETRY)
  uint8_t telemetry = 0;
  uint8_t telemetry_auto = 0;
#endif

// ******************
// rc functions
// ******************
#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

int16_t rcData[RC_CHANS];    // interval [1000;2000]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data
int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
int16_t motor[8];
int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1000};

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[2], dynD8[2];

global_conf_t global_conf;

conf_t conf;

// **********************
// GPS common variables
// **********************
  int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
  int32_t  GPS_coord[2];
  int32_t  GPS_home[2];
  int32_t  GPS_hold[2];
  uint8_t  GPS_numSat;
  uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
  int16_t  GPS_directionToHome;                         // direction to home - unit: degree
  uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
  uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
  uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10
  uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
  uint8_t  GPS_Enable  = 0;

  // The desired bank towards North (Positive) or South (Negative) : latitude
  // The desired bank towards East (Positive) or West (Negative)   : longitude
  int16_t  nav[2];
  int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

  uint8_t alarmArray[16];           // array

#if BARO
  int32_t baroPressure;
  int32_t baroTemperature;
  int32_t baroPressureSum;
#endif

void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp>>7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp-(tmp2<<7)) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2])>>7);
      prop1 = 128-((uint16_t)conf.rollPitchRate*tmp>>9); // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = (uint16_t)prop1*prop2>>7; // prop1: max is 128   prop2: max is 128   result prop1: max is 128
      dynP8[axis] = (uint16_t)conf.pid[axis].P8*prop1>>7; // was /100, is /128 now
      dynD8[axis] = (uint16_t)conf.pid[axis].D8*prop1>>7; // was /100, is /128 now
    } else {      // YAW
      rcCommand[axis] = tmp;
    }
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*2559/(2000-MINCHECK); // [MINCHECK;2000] -> [0;2559]
  tmp2 = tmp/256; // range [0;9]
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*256) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 256; // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]

  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

  if ( currentTime_us > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime_us + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  #if !(defined(SPEKTRUM) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if Spektrum Sat in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
    serialCom();
  #endif

}

uint16_t startTime = 0;
unsigned long ave_index = 0;
unsigned int ave_div = 200;
float ave_sum  = 0;

ISR(TIMER2_COMPA_vect){//timer1 interrupt 8kHz toggles pin 9
  if(digitalRead(5) == LOW)
  {     
  	if(!startTime) startTime = micros();
        //debug[1]++;
  }
  else
  { 
  	if(startTime)
  	{
  		heightMeas = micros() - startTime;
                debug[0] = heightMeas;
  		startTime = 0;
                if(ave_index==ave_div) {debug[2]=ave_sum/ave_div;ave_index=0;ave_sum=0;}
                else {ave_sum+=heightMeas;ave_index++;}
  		//heightMeasCounter++;
                //debug[2]++;
  
  	}
  }
}

void setupTimers(void)
{
  cli();//stop interrupts

  //set timer2 interrupt at 8kHz // WT, it is actually 4kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 24;//24(40kHz), 30(32kHz), 61(16kHz), 124(8kHz), 249(4kHz) = (16*10^6) / (8000*8) - 1 (must be <256) //WT, =(16*10^6)/(2*4000*8)-1
  //OCR2A = 199; // WT, =(16*10^6)/(2*40000), 40kHz, seems not working on no prescaling
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);
  //TCCR2B |= (1 << CS20); // no prescaling, seems CS20=1 not work
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);


  sei();//allow interrupts
}


void setup() {
  #if !defined(GPS_PROMINI)
    SerialOpen(0,SERIAL0_COM_SPEED);
  #endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  readGlobalSet();
  global_conf.currentSet=0;
  while(1) {                                                    // check settings integrity
    readEEPROM();                                               // check current setting integrity
    if(global_conf.currentSet == 0) break;                      // all checks is done
    global_conf.currentSet--;                                   // next setting for check
  }
  readGlobalSet();                              // reload global settings for get last profile number
  readEEPROM();                                 // load setting data from last used profile
  blinkLED(2,40,global_conf.currentSet+1);
  configureReceiver();
  initSensors();
  previousTime_us = micros();
  calibratingG = 512;
  calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
  /************************************/
  /************************************/

  f.SMALL_ANGLES_25=1; // important for gyro only conf
  debugmsg_append_str("initialization completed\n");

  //mwm: adding Davin's sonar hack
  //pinMode(6, INPUT);
  //pinMode(13,OUTPUT);
  //  attachInterrupt(5, sonarTimer, CHANGE);
  setupTimers();
}



void go_arm() {
  if(calibratingG == 0
    && f.ACC_CALIBRATED
    && failsafeCnt < 2
    ) {
    f.ARMED = 1;
  } else if(!f.ARMED) {
    blinkLED(2,255,1);
    alarmArray[8] = 1;
  }
}


void go_disarm() {
  f.ARMED = 0;
}


//mwm: this is some code I use to analyse what's happening in the system.
// I can send out three int16_t over the serial connection, to plot using MultiWiiConfig app.
// NOTE: I have disabled reading the magnetometer elsewhere in the code, for this to work.
void debug_out_over_mag(int16_t d1, int16_t d2, int16_t d3)
{
	//note: magic factor three, to make numbers come through correctly.
  imu.magADC[0] = d1*3;
  imu.magADC[1] = d2*3;
	imu.magADC[2] = d3*3;
}

// ******** Main Loop *********
void loop () {
	//runs at 400 Hz
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta;
  int16_t PTerm = 0,ITerm = 0,DTerm, PTermACC, ITermACC;
  static int16_t lastGyro[2] = {0,0};
  static int16_t errorAngleI[2] = {0,0};
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;
  int16_t rc;
  int32_t prop = 0;

  #if defined(SPEKTRUM)
    if (spekFrameFlags == 0x01) readSpektrum();
  #endif

  #if defined(OPENLRSv2MULTI)
    Read_OpenLRS_RC();
  #endif

  if (currentTime_us > rcTime) { // 50Hz
    rcTime = currentTime_us + 20000;
    computeRC();
    // Failsafe routine - added by MIS
  #if defined(FAILSAFE)
    if (failsafeCnt > (5 * FAILSAFE_DELAY) && f.ARMED) { // Stabilize, and set Throttle to specified level
      for (i = 0; i < 3; i++)
        rcData[i] = MIDRC; // after specified guard time after RC signal is lost (in 0.1sec)
      rcData[THROTTLE] = conf.failsafe_throttle;
      if (failsafeCnt > 5 * (FAILSAFE_DELAY + FAILSAFE_OFF_DELAY)) { // Turn OFF motors after specified Time (in 0.1sec)
        go_disarm(); // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
        f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
      }
      failsafeEvents++;
    }
    if (failsafeCnt > (5 * FAILSAFE_DELAY) && !f.ARMED) { //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
      go_disarm(); // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
      f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
    }
    failsafeCnt++;
  #endif
    // end of failsafe routine - next change is made with RcOptions setting

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    uint8_t stTmp = 0;
    for (i = 0; i < 4; i++) {
      stTmp >>= 2;
      if (rcData[i] > MINCHECK)
        stTmp |= 0x80;      // check for MIN
      if (rcData[i] < MAXCHECK)
        stTmp |= 0x40;      // check for MAX
    }
    if (stTmp == rcSticks) {
      if (rcDelayCommand < 250)
        rcDelayCommand++;
    } else
      rcDelayCommand = 0;
    rcSticks = stTmp;

    // perform actions
    if (rcData[THROTTLE] <= MINCHECK) {            // THROTTLE at minimum
      errorAngleI[ROLL] = 0;
      errorAngleI[PITCH] = 0;
      if (conf.activate[BOXARM] > 0) {             // Arming/Disarming via ARM BOX
        if (rcOptions[BOXARM] && f.OK_TO_ARM)
          go_arm();
        else if (f.ARMED)
          go_disarm();
      }
    }
    if (rcDelayCommand == 20) {
      if (f.ARMED) {                   // actions during armed
  #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        if (conf.activate[BOXARM]
            == 0&& rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)
          go_disarm();    // Disarm via YAW
  #endif
      } else {                        // actions during not armed
        i = 0;
        if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {   // GYRO calibration
          calibratingG = 512;
  #if BARO
          calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
  #endif
        }
        if (rcSticks == THR_LO + YAW_HI + PIT_HI + ROL_CE) {   // Enter LCD config
  #if defined(LCD_CONF)
          configurationLoop(); // beginning LCD configuration
  #endif
          previousTime_us = micros();
        }
  #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        else if (conf.activate[BOXARM]
            == 0&& rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)
          go_arm();      // Arm via YAW
  #endif
  #if ACC
        else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
          calibratingA = 512;     // throttle=max, yaw=left, pitch=min
  #endif
  #if MAG
        else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE)
          f.CALIBRATE_MAG = 1;  // throttle=max, yaw=right, pitch=min
  #endif
        i = 0;
        if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
          conf.angleTrim[PITCH] += 2;
          i = 1;
        } else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
          conf.angleTrim[PITCH] -= 2;
          i = 1;
        } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
          conf.angleTrim[ROLL] += 2;
          i = 1;
        } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
          conf.angleTrim[ROLL] -= 2;
          i = 1;
        }
        if (i) {
          writeParams(1);
          rcDelayCommand = 0;    // allow autorepetition
        }
      }
    }
    uint16_t auxState = 0;
    for (i = 0; i < 4; i++)
    {
      auxState |= (rcData[AUX1 + i] < 1300) << (3 * i)
          | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1)
          | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
    }
    for (i = 0; i < CHECKBOXITEMS; i++)
    {
      rcOptions[i] = (auxState & conf.activate[i]) > 0;
    }

    if (rcOptions[BOXARM] == 0)
    {
      f.OK_TO_ARM = 1;
    }

  }
	else { // not in rc loop
    static uint8_t taskOrder = 0; // never call all functions in the same loop, to avoid high delay spikes
    if (taskOrder > 4)
      taskOrder -= 5;
    switch (taskOrder) {
    case 0:
      taskOrder++;
      /* no break */
      /*mwm: skip this for now
      if (Mag_getADC())
        break; // max 350 µs (HMC5883) // only break when we actually did something
        */
    case 1:
      taskOrder++;
      /* no break */
      if (Baro_update() != 0)
      {
        break;
      }
      /* no break */
    case 2:
      taskOrder++;
      if (getEstimatedAltitude() != 0)
      {
        break;
      }
      /* no break */
    case 3:
      taskOrder++;
      /* no break */
    case 4:
      taskOrder++;
      break;
    }
  }

  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime_us = micros();
  cycleTime_us = currentTime_us - previousTime_us;
  previousTime_us = currentTime_us;

  for(i=0; i<8; i++)
  {
  	motor[i] = MINCOMMAND;
  }

  ///////////////////////
  //ATTITUDE ESTIMATION//
  ///////////////////////

  // A gyro reading of 8192 corresponds to 2000 deg / s
  //cycle time is in us
  //the below convert the gyro reading from integers to floating point values, in [rad/s]
#define GYRO_SCALING_TO_RAD_per_S (0.00426105774412678119f)
#define GYRO_SCALING_TO_RAD_per_uS (0.00426105774412678119f*1e-6)
  float dYaw = (imu.gyroData[YAW]*int32_t(cycleTime_us))*GYRO_SCALING_TO_RAD_per_uS;  //change in yaw angle [rad]
  estYawAngle += dYaw;
  //unwrap yaw angle (i.e. keep between +/- 180 degrees)
#define WRAP_ANGLE (3.14159265358979323846f)  //pi rad = 180 deg
  if(estYawAngle > WRAP_ANGLE){
  	estYawAngle -= 2*WRAP_ANGLE;
  }
  else if(estYawAngle < -WRAP_ANGLE){
  	estYawAngle += 2*WRAP_ANGLE;
  }

  //this constant defines how quickly the state estimator 'forgets'.
  // Necessary to get rid of initialisation errors, floating point errors, gyro noise & biases.
#define NORMAL_LEAKING_CONST (0.5f*1e-6f) //*1e-6 to get to us leak Leak
  //Rates (per microsecond!)
  float dnormalx = + (imu.gyroData[YAW]  )*GYRO_SCALING_TO_RAD_per_uS*normaly \
  		                - (imu.gyroData[PITCH])*GYRO_SCALING_TO_RAD_per_uS*normalz \
											- NORMAL_LEAKING_CONST*normalx;

  float dnormaly = - (imu.gyroData[YAW]  )*GYRO_SCALING_TO_RAD_per_uS*normalx \
  		                + (imu.gyroData[ROLL ])*GYRO_SCALING_TO_RAD_per_uS*normalz \
											- NORMAL_LEAKING_CONST*normaly;

  float dnormalz =   (imu.gyroData[PITCH])*GYRO_SCALING_TO_RAD_per_uS*normalx \
  		                - (imu.gyroData[ROLL ])*GYRO_SCALING_TO_RAD_per_uS*normaly \
											- NORMAL_LEAKING_CONST*(normalz - 1.f);

  normalx += dnormalx*int32_t(cycleTime_us);
  normaly += dnormaly*int32_t(cycleTime_us);
  normalz += dnormalz*int32_t(cycleTime_us);


  //Joystick command scaling:
#define CMD_SCALING_YAW_CORRECTION (3.14f/100*1e-6) //*1e-6 because time is in us
#define CMD_SCALING_NORMAL (1/400.f)

  //user input yaw correction, disabled for the quad
#if not(defined(SPINNER_TYPE_QUAD))
#define YAW_ZERO 0
  estYawAngle += (rcCommand[YAW] - YAW_ZERO)*CMD_SCALING_YAW_CORRECTION*int32_t(cycleTime_us);
#endif

  //The user inputs a desired normal, in the world frame (i.e. not spinning). The vehicle will then
  //rotate this into the body-fixed frame, to compare to the vehicle's estimated normal.

  //Compute the desired normal from RC:
  float desNormal_inertialX =-rcCommand[PITCH]*CMD_SCALING_NORMAL;
  float desNormal_inertialY = rcCommand[ROLL]* CMD_SCALING_NORMAL;

  const float sinYaw = sinf(estYawAngle);
  const float cosYaw = cosf(estYawAngle);

  //commanded normal, in body frame
  float desNormalx = cosYaw*desNormal_inertialX + sinYaw*desNormal_inertialY;
  float desNormaly =-sinYaw*desNormal_inertialX + cosYaw*desNormal_inertialY;

  ////////////////////
  //HEIGHT ESTIMATOR//
  ////////////////////

  if(heightMeasCounter != lastHeightMeasCounter)
  {
  	//have a new height measurement
  	uint16_t meas_dt_us = micros() - lastHeightMeasTime_us;
  	lastHeightMeasTime_us = micros();
#define LP_FILTER_HEIGHT (0.5f)
#define LP_FILTER_VELOCITY (0.5f)
  	estHeight = LP_FILTER_HEIGHT*estHeight + (1-LP_FILTER_HEIGHT)*heightMeas;
    heightMeasDiff = (heightMeas - lastHeightMeas);
  	estVelocity = LP_FILTER_VELOCITY*estVelocity + (1-LP_FILTER_VELOCITY)*(int32_t(heightMeas) - int32_t(lastHeightMeas))*1e6f/meas_dt_us;

  	lastHeightMeasCounter = heightMeasCounter;
    lastHeightMeas = heightMeas;
  }


  //////////////
  //CONTROLLER//
  //////////////

#define VEHICLE_WEIGHT_N 0.353f  // the vehicle's weight, in N
#define THRUST_OFFSET (1200) // an RC value below this maps to zero.
#define THRUST_CHANNEL_MAX (1000) //after removing offset

	//state errors: the controller will try to drive these to zero
	float errNormal_x = normalx - desNormalx;
	float errNormal_y = normaly - desNormaly;
	float errRates_x  = imu.gyroData[ROLL]*GYRO_SCALING_TO_RAD_per_S;
	float errRates_y  = imu.gyroData[PITCH]*GYRO_SCALING_TO_RAD_per_S;

	//These files include the controller gains.
	// To change the controller gains, execute the Python script "generateController.py"
#if defined(SPINNER_TYPE_TRI)
#include "ControllerGains_tri.h"
#elif defined(SPINNER_TYPE_QUAD)
#include "ControllerGains_quad.h"
#endif

	//compute the force differentials:
	float dfx  = - GAIN_LQR_DFX_RATES_X*errRates_x   - GAIN_LQR_DFX_RATES_Y*errRates_y \
               - GAIN_LQR_DFX_NORMAL_X*errNormal_x - GAIN_LQR_DFX_NORMAL_Y*errNormal_y;
	float dfy  = - GAIN_LQR_DFY_RATES_X*errRates_x   - GAIN_LQR_DFY_RATES_Y*errRates_y \
               - GAIN_LQR_DFY_NORMAL_X*errNormal_x - GAIN_LQR_DFY_NORMAL_Y*errNormal_y;
	float fsum = float(rcCommand[THROTTLE] - THRUST_OFFSET) * float(2*VEHICLE_WEIGHT_N/THRUST_CHANNEL_MAX);  //total force
	//correct for angle:
	if(normalz>0.5f){
		//this is the same as dividing by the cosine of the roll & pitch angles
		fsum /= normalz;
	}

  if(f.ARMED)
  {
  	if(rcCommand[THROTTLE] > THRUST_OFFSET)
    {
  		//minimum throttle exceeded, can mix commands to motors:
#if defined(SPINNER_TYPE_TRI)
      set_motor_thrusts_trispinner(fsum, dfx, dfy);
#elif defined(SPINNER_TYPE_QUAD)
      set_motor_thrusts_quadspinner(fsum, dfx, dfy);
#endif
    }
  	else
  	{
  		//are below minimum throttle threshold.
      set_all_motors_off();
  	}

  //show if we're close to zero yaw by blinking the LED
#define INDICATION_ANGLE_RAD 0.174f
    if(fabsf(estYawAngle) < INDICATION_ANGLE_RAD) writeIndicatorLED(1);
    else writeIndicatorLED(0);
  }
  else
  {
  	//not armed.
    set_all_motors_off();
    //reset yaw estimate, and normal estimate
    estYawAngle = 0;
    normalx = normaly = 0;
    normalz = 1;
    //read from pin 5
  }

	debug_out_over_mag(int16_t(heightMeas), int16_t(estHeight),  int16_t(estVelocity/100));

  //debug outputs happen here:
//#if 1
//  //get the yaw rate
//  static int16_t  maxYawGyro = 0;
//  maxYawGyro = max(maxYawGyro, imu.gyroADC[YAW]);
//	debug_out_over_mag(maxYawGyro, maxYawGyro/10, maxYawGyro/100);
//#else
//	debug_out_over_mag(int16_t(normalx*1000), int16_t(normaly*1000),  int16_t(normalz*1000));
//#endif

  writeMotors();
}
