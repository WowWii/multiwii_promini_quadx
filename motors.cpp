#include "motors.h"
#include "config.h"
#include <stdint.h>

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max

extern int16_t motor[8];

#define MOTOR_TRI_0 3 //front
#define MOTOR_TRI_1 2 //rear, left
#define MOTOR_TRI_2 0 //rear, right

#define MOTOR_QUAD_FR 3 //front
#define MOTOR_QUAD_LE 2 //left
#define MOTOR_QUAD_RE 0 //rear
#define MOTOR_QUAD_RI 1 //right

//these coefficients are from a data fit, see the Python script
// 'analyseMotorThrust.py'
#define coeffAx100 283// times a hundred!
#define coeffB 1274
#define FORCE_TO_CMD(f) (((f)*coeffAx100)/100 + coeffB)

void set_motor_thrusts_trispinner(float fsum, float dfx, float dfy)
{
	//Defines the commands required to produce the required forces
	uint16_t force0_mN = uint16_t(max((fsum/3.f                  - 0.666666667f*dfy)*1000, 0));
	uint16_t force1_mN = uint16_t(max((fsum/3.f + 0.4330127f*dfx + 0.3333333333*dfy)*1000, 0));
	uint16_t force2_mN = uint16_t(max((fsum/3.f - 0.4330127f*dfx + 0.3333333333*dfy)*1000, 0));

  motor[MOTOR_TRI_0] = max(min(FORCE_TO_CMD(force0_mN), MAXTHROTTLE), MINTHROTTLE);
  motor[MOTOR_TRI_1] = max(min(FORCE_TO_CMD(force1_mN), MAXTHROTTLE), MINTHROTTLE);
  motor[MOTOR_TRI_2] = max(min(FORCE_TO_CMD(force2_mN), MAXTHROTTLE), MINTHROTTLE);
}

void set_motor_thrusts_quadspinner(float fsum, float dfx, float dfy)
{
	//Defines the commands required to produce the required forces
	uint16_t forceFR_mN = uint16_t(max((fsum/4.f         - dfy/2)*1000, 0));
	uint16_t forceLE_mN = uint16_t(max((fsum/4.f + dfx/2        )*1000, 0));
	uint16_t forceRE_mN = uint16_t(max((fsum/4.f         + dfy/2)*1000, 0));
	uint16_t forceRI_mN = uint16_t(max((fsum/4.f - dfx/2        )*1000, 0));

  motor[MOTOR_QUAD_FR] = max(min(FORCE_TO_CMD(forceFR_mN), MAXTHROTTLE), MINTHROTTLE);
  motor[MOTOR_QUAD_LE] = max(min(FORCE_TO_CMD(forceLE_mN), MAXTHROTTLE), MINTHROTTLE);
  motor[MOTOR_QUAD_RE] = max(min(FORCE_TO_CMD(forceRE_mN), MAXTHROTTLE), MINTHROTTLE);
  motor[MOTOR_QUAD_RI] = max(min(FORCE_TO_CMD(forceRI_mN), MAXTHROTTLE), MINTHROTTLE);
}

void set_all_motors_off(void)
{
	for(char i=0; i<4; i++) motor[i] = MINTHROTTLE;
}
