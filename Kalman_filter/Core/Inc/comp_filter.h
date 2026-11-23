#ifndef COMP_FILTER_H
#define COMP_FILTER_H

/* Complementary filter (one instance per angle):
 *   angle = alpha * (angle + gyro*dt) + (1 - alpha) * accAngle
 *   alpha = tau / (tau + dt)
 *
 * tau (seconds) is the single parameter to tune:
 *   - above tau the accelerometer wins (slow correction of the gyro drift),
 *   - below tau the gyro wins (fast, smooth movements).
 * Works with angles in the range [-180, 180] (the roll wrap-around is handled).
 */
typedef struct {
    float angle;   /* filtered angle (deg) */
    float alpha;;     /* time constant (s)    */
} Comp_t;

void  Comp_Init(Comp_t *c, float alpha);
void  Comp_SetAngle(Comp_t *c, float angle_deg);
float Comp_Update(Comp_t *c, float accAngle_deg, float gyroRate_dps, float dt);

#endif
