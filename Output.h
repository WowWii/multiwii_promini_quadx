#ifndef OUTPUT_H_
#define OUTPUT_H_

extern uint8_t PWM_PIN[8];

void initOutput();
void mixTable();
void writeServos();
void writeMotors();
void writeIndicatorLED(uint8_t on);

#endif /* OUTPUT_H_ */
