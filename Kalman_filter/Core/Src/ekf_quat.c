#include "ekf_quat.h"
#include <math.h>

#define RAD2DEG 57.2957795f

static void q_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-6f) { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; return; }
    n = 1.0f / n;
    for (int i = 0; i < 4; i++) q[i] *= n;
}

static int inv3(const float m[3][3], float o[3][3])
{
    float a = m[0][0], b = m[0][1], c = m[0][2];
    float d = m[1][0], e = m[1][1], f = m[1][2];
    float g = m[2][0], h = m[2][1], i = m[2][2];

    float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    if (fabsf(det) < 1e-12f) return 0;
    float id = 1.0f / det;

    o[0][0] =  (e*i - f*h) * id;  o[0][1] = -(b*i - c*h) * id;  o[0][2] =  (b*f - c*e) * id;
    o[1][0] = -(d*i - f*g) * id;  o[1][1] =  (a*i - c*g) * id;  o[1][2] = -(a*f - c*d) * id;
    o[2][0] =  (d*h - e*g) * id;  o[2][1] = -(a*h - b*g) * id;  o[2][2] =  (a*e - b*d) * id;
    return 1;
}

void EKF_Init(EKF_Quat_t *e, float sigma_gyro, float sigma_acc)
{
    e->q[0] = 1.0f; e->q[1] = e->q[2] = e->q[3] = 0.0f;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            e->P[i][j] = (i == j) ? 0.1f : 0.0f;
    e->sigma_gyro = sigma_gyro;
    e->sigma_acc  = sigma_acc;
    e->acc_gate   = 0.3f;
}

/* Initial orientation from the accelerometer (yaw = 0) */
void EKF_SetFromAccel(EKF_Quat_t *e, float ax, float ay, float az)
{
    float roll  = atan2f(ay, az);
    float pitch = atanf(-ax / sqrtf(ay*ay + az*az));
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

    e->q[0] =  cr * cp;
    e->q[1] =  sr * cp;
    e->q[2] =  cr * sp;
    e->q[3] = -sr * sp;
}

/* ---- Prediction: q <- q + 0.5*dt * q (x) [0, w],  P <- F P F' + Q ---- */
void EKF_Predict(EKF_Quat_t *e, float gx, float gy, float gz, float dt)
{
    float h = 0.5f * dt;
    float F[4][4] = {
        {1.0f,  -h*gx, -h*gy, -h*gz},
        { h*gx,  1.0f,  h*gz, -h*gy},
        { h*gy, -h*gz,  1.0f,  h*gx},
        { h*gz,  h*gy, -h*gx,  1.0f}
    };

    float qn[4];
    for (int i = 0; i < 4; i++) {
        qn[i] = 0.0f;
        for (int j = 0; j < 4; j++) qn[i] += F[i][j] * e->q[j];
    }
    q_normalize(qn);

    /* P = F P F' */
    float FP[4][4], P2[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += F[i][k] * e->P[k][j];
            FP[i][j] = s;
        }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += FP[i][k] * F[j][k];
            P2[i][j] = s;
        }

    /* Q = (sigma_gyro * dt / 2)^2 * (I - q q')  : gyro noise mapped onto the quaternion */
    float s2 = e->sigma_gyro * h;
    s2 *= s2;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float qq = qn[i] * qn[j];
            e->P[i][j] = P2[i][j] + s2 * (((i == j) ? 1.0f : 0.0f) - qq);
        }

    for (int i = 0; i < 4; i++) e->q[i] = qn[i];
}

/* ---- Update: the accelerometer measures the gravity direction in the body frame ---- */
void EKF_UpdateAccel(EKF_Quat_t *e, float ax, float ay, float az)
{
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 1e-3f || fabsf(n - 1.0f) > e->acc_gate) return;   /* board is accelerating: skip */

    float z[3] = { ax / n, ay / n, az / n };

    float w = e->q[0], x = e->q[1], y = e->q[2], zq = e->q[3];

    /* predicted measurement h(q) = R(q)' * [0 0 1]' */
    float hq[3] = {
        2.0f * (x*zq - w*y),
        2.0f * (y*zq + w*x),
        w*w - x*x - y*y + zq*zq
    };

    /* Jacobian H = dh/dq (3x4) */
    float H[3][4] = {
        {-2.0f*y,  2.0f*zq, -2.0f*w,  2.0f*x},
        { 2.0f*x,  2.0f*w,   2.0f*zq, 2.0f*y},
        { 2.0f*w, -2.0f*x,  -2.0f*y,  2.0f*zq}
    };

    /* PHt = P * H'  (4x3) */
    float PHt[4][3];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += e->P[i][k] * H[j][k];
            PHt[i][j] = s;
        }

    /* S = H * PHt + R  (3x3) */
    float S[3][3], Si[3][3];
    float r = e->sigma_acc * e->sigma_acc;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += H[i][k] * PHt[k][j];
            S[i][j] = s + ((i == j) ? r : 0.0f);
        }
    if (!inv3(S, Si)) return;

    /* K = PHt * S^-1  (4x3) */
    float K[4][3];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += PHt[i][k] * Si[k][j];
            K[i][j] = s;
        }

    /* q <- q + K (z - h) */
    float y_[3] = { z[0] - hq[0], z[1] - hq[1], z[2] - hq[2] };
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++) e->q[i] += K[i][j] * y_[j];
    q_normalize(e->q);

    /* P <- (I - K H) P, then symmetrize */
    float IKH[4][4], Pn[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += K[i][k] * H[k][j];
            IKH[i][j] = ((i == j) ? 1.0f : 0.0f) - s;
        }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += IKH[i][k] * e->P[k][j];
            Pn[i][j] = s;
        }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            e->P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
}

/* quaternion -> Euler angles (ZYX convention), degrees */
void EKF_GetEuler(const EKF_Quat_t *e, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float w = e->q[0], x = e->q[1], y = e->q[2], z = e->q[3];

    float sp = 2.0f * (w*y - z*x);
    if (sp >  1.0f) sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;

    *roll_deg  = atan2f(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y)) * RAD2DEG;
    *pitch_deg = asinf(sp) * RAD2DEG;
    *yaw_deg   = atan2f(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z)) * RAD2DEG;
}
