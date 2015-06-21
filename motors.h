/* Some utilities for abstracting the motors.
 *
 * Allows us to specify motor forces in [N], and transforms automatically
 * to motor commands sent over PWM. Still uses the underlying stuff from
 * Multiwii.
 *
 * Also has the mixing tables to transform a total force, and force
 * differentials to the different motor commands.
 */
#ifndef MOTORS_H_
#define MOTORS_H_

void set_motor_thrusts_trispinner(float fsum, float dfx, float dfy);//the mixing for the 3 arm vehicle. Forces in [N].
void set_motor_thrusts_quadspinner(float fsum, float dfx, float dfy);//the mixing for the 4 arm vehicle. Forces in [N].
void set_all_motors_off(void); // sets all the motors to zero.

#endif
