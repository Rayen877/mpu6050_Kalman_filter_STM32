#ifndef EKF_QUAT_H
#define EKF_QUAT_H

/* Extended Kalman Filter on a unit quaternion q = [w, x, y, z]
 *   state       : q (orientation of the board relative to the world, body -> world)
 *   input       : gyro rates in rad/s   (prediction step)
 *   measurement : accelerometer, direction of gravity (update step)
 * No yaw correction (needs a magnetometer): yaw drifts, roll and pitch are fine.
 */
typedef struct {
    float q[4];          /* w, x, y, z                                  */
    float P[4][4];       /* state covariance                            */
    float sigma_gyro;    /* gyro noise  (rad/s)  -> process noise Q     */
    float sigma_acc;     /* accel noise (g)      -> measurement noise R */
    float acc_gate;      /* ignore accel if | |a| - 1 g | > acc_gate    */
} EKF_Quat_t;

void EKF_Init(EKF_Quat_t *e, float sigma_gyro, float sigma_acc);
void EKF_SetFromAccel(EKF_Quat_t *e, float ax, float ay, float az);
void EKF_Predict(EKF_Quat_t *e, float gx, float gy, float gz, float dt);  /* rad/s, s */
void EKF_UpdateAccel(EKF_Quat_t *e, float ax, float ay, float az);        /* g        */
void EKF_GetEuler(const EKF_Quat_t *e, float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif
