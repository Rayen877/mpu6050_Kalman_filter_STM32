#include "comp_filter.h"

static float wrap180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void Comp_Init(Comp_t *c, float alpha)
{
    c->angle = 0.0f;
    c->alpha = alpha;
}

float Comp_Update(Comp_t *c, float accAngle_deg, float gyroRate_dps, float dt)
{
    float pred = c->angle + gyroRate_dps * dt;
    float err  = wrap180(accAngle_deg - pred);

    c->angle = wrap180(pred + (1.0f - c->alpha) * err);
    return c->angle;
}

void Comp_SetAngle(Comp_t *c, float angle_deg)
{
    c->angle = angle_deg;
}
