#include "rotation_fast.h"
#include <math.h>
float comp_tau = 1/0.8f;

void update_rotation_state(RotationStateVector *state, const float *gyro, const float *accel, float dt)
{   
    float gyro_x = gyro[0];
    float gyro_y = gyro[1];
    float gyro_z = gyro[2];
    float yaw_value = state->ux*gyro_x + state->uy*gyro_y + state->uz*gyro_z;
    float gyro_rollpitch_x = gyro_x - state->ux * yaw_value;
    float gyro_rollpitch_y = gyro_y - state->uy * yaw_value;
    float gyro_rollpitch_z = gyro_z - state->uz * yaw_value;

    float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    float wcross_vector[3] = {gyro_rollpitch_z*state->uy - gyro_rollpitch_y*state->uz,
                              gyro_rollpitch_x*state->uz - gyro_rollpitch_z*state->ux,
                              gyro_rollpitch_y*state->ux - gyro_rollpitch_x*state->uy};
    state->phi += yaw_value * dt;

    state->ux += wcross_vector[0] * dt;
    state->uy += wcross_vector[1] * dt;
    state->uz += wcross_vector[2] * dt;
    
    if (accel_norm>8 && accel_norm<12) {
        state->ux += (accel[0]/accel_norm-state->ux)*comp_tau * dt;
        state->uy += (accel[1]/accel_norm-state->uy)*comp_tau * dt;
        state->uz += (accel[2]/accel_norm-state->uz)*comp_tau * dt;
    }
    normalize_adjust_state(state);
}

void normalize_adjust_state(RotationStateVector *v)
{
    /* A normalization method based on successive adjustments*/
    float norm = v->ux * v->ux + v->uy * v->uy + v->uz * v->uz;
    float norm_diff = 1.0f - norm;
    float gain = 1.0f;
    if (fabsf(norm_diff) > 1e-12)
    {
        gain = 1.0f+ norm_diff*0.5f;
        if (gain < 0.01f) gain = 0.01f;
        v->ux *= gain;
        v->uy *= gain;
        v->uz *= gain;
    } 
}