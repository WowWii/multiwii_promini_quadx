#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

void initializeSoftPWM(void);

#if defined(SERVO)
void initializeServo();
#endif

/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12}; //for a quad+: rear,right,left,front
#endif

/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/
#if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
#if (NUMBER_MOTOR > 4)
//for HEX Y6 and HEX6/HEX6X/HEX6H flat for promini
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;
#endif
#if (NUMBER_MOTOR > 6)
//for OCTO on promini
volatile uint8_t atomicPWM_PINA2_lowState;
volatile uint8_t atomicPWM_PINA2_highState;
volatile uint8_t atomicPWM_PIN12_lowState;
volatile uint8_t atomicPWM_PIN12_highState;
#endif
#else
#if (NUMBER_MOTOR > 4)
//for HEX Y6 and HEX6/HEX6X/HEX6H and for Promicro
volatile uint16_t atomicPWM_PIN5_lowState;
volatile uint16_t atomicPWM_PIN5_highState;
volatile uint16_t atomicPWM_PIN6_lowState;
volatile uint16_t atomicPWM_PIN6_highState;
#endif
#if (NUMBER_MOTOR > 6)
//for OCTO on Promicro
volatile uint16_t atomicPWM_PINA2_lowState;
volatile uint16_t atomicPWM_PINA2_highState;
volatile uint16_t atomicPWM_PIN12_lowState;
volatile uint16_t atomicPWM_PIN12_highState;
#endif
#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
	/********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
#if defined(PROMINI)//this is us
  #if (NUMBER_MOTOR > 0)
    #ifndef EXT_MOTOR_RANGE
    	OCR1A = motor[0]>>3; //  pin 9
    #else
    	OCR1A = ((motor[0]>>2) - 250);
    #endif
  #endif
  #if (NUMBER_MOTOR > 1)
    #ifndef EXT_MOTOR_RANGE
    	OCR1B = motor[1]>>3; //  pin 10
    #else
    	OCR1B = ((motor[1]>>2) - 250);
    #endif
  #endif
  #if (NUMBER_MOTOR > 2)
    #ifndef EXT_MOTOR_RANGE
    	OCR2A = motor[2]>>3; //  pin 11
    #else
    	OCR2A = ((motor[2]>>2) - 250);
    #endif
  #endif
  #if (NUMBER_MOTOR > 3)
    #ifndef EXT_MOTOR_RANGE
    	OCR2B = motor[3]>>3; //  pin 3
    #else
    	OCR2B = ((motor[3]>>2) - 250);
    #endif
  #endif
#endif
}

/**************************************************************************************/
/************          Turn the indicator LED on or off              ******************/
/**************************************************************************************/
void writeIndicatorLED(uint8_t on)
{
  digitalWrite(13, on?HIGH:LOW); //For WS board
//  digitalWrite(6, on?HIGH:LOW); //for HK board
  
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
	for (uint8_t i = 0; i < NUMBER_MOTOR; i++) {
		motor[i] = mc;
	}
	writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
	/****************            mark all PWM pins as Output             ******************/
	for (uint8_t i = 0; i < 4; i++) {
		pinMode(PWM_PIN[i], OUTPUT);
	}

	/********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
#if defined(PROMINI)
  #if (NUMBER_MOTOR > 0)
  	TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
  #endif
  #if (NUMBER_MOTOR > 1)
  	TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
  #endif
  #if (NUMBER_MOTOR > 2)
  	TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
  #endif
  #if (NUMBER_MOTOR > 3)
  	TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
  #endif
#endif

	/********  special version of MultiWii to calibrate all attached ESCs ************/
#if defined(ESC_CALIB_CANNOT_FLY)
	writeAllMotors(ESC_CALIB_HIGH);
	blinkLED(2,20, 2);
	delay(4000);
	writeAllMotors(ESC_CALIB_LOW);
	blinkLED(3,20, 2);
	while (1) {
		delay(5000);
		blinkLED(4,20, 2);
#if defined(BUZZER)
		alarmArray[7] = 2;
#endif
	}
	exit; // statement never reached
#endif

	writeAllMotors(MINCOMMAND);
	delay(300);
#if defined(SERVO)
	initializeServo();
#endif
}

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/

void mixTable() {
	int16_t maxMotor;
	uint8_t i;
#if defined(DYNBALANCE)
	return;
#endif
#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
#define SERVODIR(n,b) ((conf.servoConf[n].rate & b) ? -1 : 1)

	/****************                   main Mix Table                ******************/
	//quad mixing
	motor[0] = PIDMIX(-1, +1, -1); //REAR_R
	motor[1] = PIDMIX(-1, -1, +1); //FRONT_R
	motor[2] = PIDMIX(+1, +1, +1); //REAR_L
	motor[3] = PIDMIX(+1, -1, -1); //FRONT_L

	/****************                compensate the Motors values                ******************/
#ifdef VOLTAGEDROP_COMPENSATION
	{
#if (VBATNOMINAL == 126)
#define GOV_R_NUM 36
		static int8_t g[] = {0,3,5,8,11,14,17,19,22,25,28,31,34,38,41,44,47,51,54,58,61,65,68,72,76,79,83,87,91,95,99,104,108,112,117,121,126};
#elif (VBATNOMINAL == 84)
#define GOV_R_NUM 24
		static int8_t g[] = {0,4,8,12,17,21,25,30,34,39,44,49,54,59,65,70,76,81,87,93,99,106,112,119,126};
#else
#error "VOLTAGEDROP_COMPENSATION requires correction values which fit VBATNOMINAL; not yet defined for your value of VBATNOMINAL"
#endif
		uint8_t v = constrain( VBATNOMINAL - constrain(analog.vbat, conf.vbatlevel_crit, VBATNOMINAL), 0, GOV_R_NUM);
		for (i = 0; i < NUMBER_MOTOR; i++) {
			motor[i] += ( ( (int32_t)(motor[i]-1000) * (int32_t)g[v] ) )/ 500;
		}
	}
#endif
	/****************                normalize the Motors values                ******************/
	maxMotor = motor[0];
	for (i = 1; i < NUMBER_MOTOR; i++) {
		if (motor[i] > maxMotor) {
			maxMotor = motor[i];
		}
	}
	for (i = 0; i < NUMBER_MOTOR; i++) {
		if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
			motor[i] -= maxMotor - MAXTHROTTLE;
		motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
		if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
#ifndef MOTOR_STOP
			motor[i] = conf.minthrottle;
#else
		motor[i] = MINCOMMAND;
#endif
		if (!f.ARMED)
			motor[i] = MINCOMMAND;
	}

}
