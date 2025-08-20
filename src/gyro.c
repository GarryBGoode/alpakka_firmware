// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2022, Input Labs Oy.

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "button.h"
#include "config.h"
#include "gyro.h"
#include "common.h"
#include "hid.h"
#include "imu.h"
#include "pin.h"
#include "touch.h"
#include "vector.h"
#include "rotation_fast.h"

float sensitivity_multiplier;

uint8_t world_init = 0;
Vector world_top;
Vector world_fw;
Vector world_right;
Vector accel_smooth;
RotationStateVector rotation_state;
Vector gyro_act;
Vector gyro_corr;
Vector gyro_offset_local = {0, 0, 0};

void gyro_update_sensitivity() {
    uint8_t preset = config_get_mouse_sens_preset();
    sensitivity_multiplier = config_get_mouse_sens_value(preset);
}

void gyro_accel_correction() {
    Vector accel = imu_read_accel();
    // Convert to inverted unit value.
    accel.x /= -BIT_14;
    accel.y /= -BIT_14;
    accel.z /= -BIT_14;
    // Get a smoothed gravity vector.
    accel_smooth = vector_smooth(accel_smooth, accel, CFG_ACCEL_CORRECTION_SMOOTH);
    if (world_init < CFG_ACCEL_CORRECTION_SMOOTH) {
        // It the world space orientation is not fully initialized.
        world_top = vector_normalize(vector_invert(accel_smooth));
        world_fw = vector_cross_product(world_top, (Vector){1, 0, 0});
        world_right = vector_cross_product(world_fw, world_top);
        world_init++;
    } else {
        // Correction.
        float rate_fw = (world_right.z - accel_smooth.x) * CFG_ACCEL_CORRECTION_RATE;
        float rate_r = (world_fw.z - accel_smooth.y) * CFG_ACCEL_CORRECTION_RATE;
        Vector4 correction_fw = quaternion(world_fw, rate_fw);
        Vector4 correction_r = quaternion(world_right, -rate_r);
        Vector4 correction = qmultiply(correction_fw, correction_r);
        world_top = qrotate(correction, world_top);
        world_right = qrotate(correction, world_right);
        world_fw = vector_cross_product(world_top, world_right);
    }
}

void gyro_absolute_output(float value, uint8_t *actions, bool *pressed) {
    for(uint8_t i=0; i<4; i++) {
        uint8_t action = actions[i];
        if (hid_is_axis(action)) {
            value = fabs(value);
            if      (action == GAMEPAD_AXIS_LX)     hid_gamepad_axis(LX,  value);
            else if (action == GAMEPAD_AXIS_LY)     hid_gamepad_axis(LY,  value);
            else if (action == GAMEPAD_AXIS_LZ)     hid_gamepad_axis(LZ,  value);
            else if (action == GAMEPAD_AXIS_RX)     hid_gamepad_axis(RX,  value);
            else if (action == GAMEPAD_AXIS_RY)     hid_gamepad_axis(RY,  value);
            else if (action == GAMEPAD_AXIS_RZ)     hid_gamepad_axis(RZ,  value);
            else if (action == GAMEPAD_AXIS_LX_NEG) hid_gamepad_axis(LX, -value);
            else if (action == GAMEPAD_AXIS_LY_NEG) hid_gamepad_axis(LY, -value);
            else if (action == GAMEPAD_AXIS_LZ_NEG) hid_gamepad_axis(LZ, -value);
            else if (action == GAMEPAD_AXIS_RX_NEG) hid_gamepad_axis(RX, -value);
            else if (action == GAMEPAD_AXIS_RY_NEG) hid_gamepad_axis(RY, -value);
            else if (action == GAMEPAD_AXIS_RZ_NEG) hid_gamepad_axis(RZ, -value);
        } else {
            if (!(*pressed) && value >= 0.5) {
                hid_press(action);
                if (i==3) *pressed = true;
            }
            else if (*pressed && value < 0.5) {
                hid_release(action);
                if (i==3) *pressed = false;
            }
        }
    }
}

void gyro_incremental_output(float value, uint8_t *actions) {
    for(uint8_t i=0; i<4; i++) {
        uint8_t action = actions[i];
        if      (action == MOUSE_X)     hid_mouse_move(value, 0);
        else if (action == MOUSE_Y)     hid_mouse_move(0, value);
        else if (action == MOUSE_X_NEG) hid_mouse_move(-value, 0);
        else if (action == MOUSE_Y_NEG) hid_mouse_move(0, -value);
    }
}

void gyro_mouse_output(float value, uint8_t *actions) {
    for(uint8_t i=0; i<4; i++) {
        uint8_t action = actions[i];
        if      (action == MOUSE_X)     hid_mouse_move(value, 0);
        else if (action == MOUSE_Y)     hid_mouse_move(0, value);
        else if (action == MOUSE_X_NEG) hid_mouse_move(-value, 0);
        else if (action == MOUSE_Y_NEG) hid_mouse_move(0, -value);
    }
}

float hssnf(float t, float k, float x) {
    float a = x - (x * k);
    float b = 1 - (x * k * (1/t));
    return a / b;
}

void Gyro_update_world()
{
    static uint32_t time_us_lock_prev = 0;
    uint32_t time_us = time_us_32();
    uint32_t dt_us = time_us - time_us_lock_prev; // Safe for overflow with unsigned arithmetic
    time_us_lock_prev = time_us;
    Vector gyro = imu_read_gyro();
    Vector accel = imu_read_accel();

    // gyro axis convention is different from physical IMU axis convention.
    float gyro_arr[3] = {gyro.y * GYRO_SENS_RADPS_500, 
                         gyro.z * GYRO_SENS_RADPS_500, 
                        -gyro.x * GYRO_SENS_RADPS_500};
    gyro_act.x = gyro_arr[0];
    gyro_act.y = gyro_arr[1];
    gyro_act.z = gyro_arr[2];
    gyro_corr.x = gyro_arr[0] - gyro_offset_local.x;
    gyro_corr.y = gyro_arr[1] - gyro_offset_local.y;
    gyro_corr.z = gyro_arr[2] - gyro_offset_local.z;
    gyro_arr[0] = gyro_corr.x;
    gyro_arr[1] = gyro_corr.y;
    gyro_arr[2] = gyro_corr.z;
    // Acceleration axis convention is unchanged from physical IMU axis convention.
    float accel_arr[3] = {accel.x * ACCEL_SENS_2G, 
                          accel.y * ACCEL_SENS_2G, 
                          accel.z * ACCEL_SENS_2G};
    update_rotation_state(&rotation_state, gyro_arr, accel_arr, dt_us / 1000000.0f);

}

void Gyro_check_offset()
{
    static uint32_t rest_count = 0;
    Config *config = config_read();
    float gyro_size = (gyro_act.x * gyro_act.x + gyro_act.y * gyro_act.y + gyro_act.z * gyro_act.z);
    float var_size_0 = (config->stddev_gyro_0_x * config->stddev_gyro_0_x + 
                        config->stddev_gyro_0_y * config->stddev_gyro_0_y + 
                        config->stddev_gyro_0_z * config->stddev_gyro_0_z)* GYRO_SENS_RADPS_500 * GYRO_SENS_RADPS_500 ;
    // float var_size_1 = (config->stddev_gyro_1_x * config->stddev_gyro_1_x + 
    //                     config->stddev_gyro_1_y * config->stddev_gyro_1_y + 
    //                     config->stddev_gyro_1_z * config->stddev_gyro_1_z)* GYRO_SENS_RADPS_125 * GYRO_SENS_RADPS_125 ;
    // float var_size = (var_size_0 * var_size_1) / (var_size_0 + var_size_1) ;

    if(gyro_size < var_size_0*10) {
        rest_count++;
        if (rest_count >= 1000) {
            rest_count = 1000;
            gyro_offset_local.x += (gyro_act.x-gyro_offset_local.x) / 2e3;
            gyro_offset_local.y += (gyro_act.y-gyro_offset_local.y) / 2e3;
            gyro_offset_local.z += (gyro_act.z-gyro_offset_local.z) / 2e3;
        }
    } else {
        rest_count = 0;
    }

}

void Gyro__report_absolute_fast(Gyro *self){

    // Output calculation.
    // physically, X points to the right of the controller
    float x = (atan2f(rotation_state.ux,rotation_state.uz)) / M_PI;
    float y = -(asinf(rotation_state.uy)) / M_PI;
    float z = rotation_state.phi / M_PI;

    x = constrain(x * 1.1, -1, 1); // Additional saturation.
    x = ramp(x, self->absolute_x_min/180, self->absolute_x_max/180); // Adjust range.
    y = ramp(y, self->absolute_y_min/180, self->absolute_y_max/180); // Adjust range.
    // Output mapping.
    if (x >= 0) gyro_absolute_output( x, self->actions_x_pos, &(self->pressed_x_pos));
    else        gyro_absolute_output(-x, self->actions_x_neg, &(self->pressed_x_neg));
    if (y >= 0) gyro_absolute_output( y, self->actions_y_pos, &(self->pressed_y_pos));
    else        gyro_absolute_output(-y, self->actions_y_neg, &(self->pressed_y_neg));
    if (z >= 0) gyro_absolute_output( z, self->actions_z_pos, &(self->pressed_z_pos));
    else        gyro_absolute_output(-z, self->actions_z_neg, &(self->pressed_z_neg));
    
}


void Gyro__report_incremental(Gyro *self) {
    static float sub_x = 0;
    static float sub_y = 0;
    static float sub_z = 0;
     // Read gyro values.
    Vector imu_gyro = imu_read_gyro();
    float x = imu_gyro.x * CFG_GYRO_SENSITIVITY_X * sensitivity_multiplier;
    float y = imu_gyro.y * CFG_GYRO_SENSITIVITY_Y * sensitivity_multiplier;
    float z = imu_gyro.z * CFG_GYRO_SENSITIVITY_Z * sensitivity_multiplier;


    // compensate tick frequency.
    x *= (float)REFERENCE_TICK_FREQUENCY/(float)CFG_TICK_FREQUENCY;
    y *= (float)REFERENCE_TICK_FREQUENCY/(float)CFG_TICK_FREQUENCY;
    z *= (float)REFERENCE_TICK_FREQUENCY/(float)CFG_TICK_FREQUENCY;

    //Additional processing.
    float t = CFG_IMU_DEADZONE*0;
    float k = CFG_IMU_DEADZONE_STRENGTH;
    if      (x > 0 && x <  t) x =  hssnf(t, k,  x);
    else if (x < 0 && x > -t) x = -hssnf(t, k, -x);
    if      (y > 0 && y <  t) y =  hssnf(t, k,  y);
    else if (y < 0 && y > -t) y = -hssnf(t, k, -y);
    if      (z > 0 && z <  t) z =  hssnf(t, k,  z);
    else if (z < 0 && z > -t) z = -hssnf(t, k, -z);

    // Reintroduce subpixel leftovers.
    x += sub_x;
    y += sub_y;
    z += sub_z;
    // Round down and save leftovers.
    sub_x = modff(x, &x);
    sub_y = modff(y, &y);
    sub_z = modff(z, &z);
    // Report.
    if (x >= 0) gyro_incremental_output( x, self->actions_x_pos);
    else        gyro_incremental_output(-x, self->actions_x_neg);
    if (y >= 0) gyro_incremental_output( y, self->actions_y_pos);
    else        gyro_incremental_output(-y, self->actions_y_neg);
    if (z >= 0) gyro_incremental_output( z, self->actions_z_pos);
    else        gyro_incremental_output(-z, self->actions_z_neg);
}

void Gyro__report_incremental_rot_based(Gyro *self) {
    static RotationStateVector rotation_ref = {0, 0, 1, 0};
    float sens_x = CFG_GYRO_SENSITIVITY_X * sensitivity_multiplier /GYRO_SENS_RADPS_500*REFERENCE_TICK_FREQUENCY;
    float sens_y = CFG_GYRO_SENSITIVITY_Y * sensitivity_multiplier /GYRO_SENS_RADPS_500*REFERENCE_TICK_FREQUENCY;
    float sens_z = CFG_GYRO_SENSITIVITY_Z * sensitivity_multiplier /GYRO_SENS_RADPS_500*REFERENCE_TICK_FREQUENCY;
    bool active = (self->mode == GYRO_MODE_TOUCH_ON && Gyro__is_engaged(self)) ||
            (self->mode == GYRO_MODE_TOUCH_OFF && !Gyro__is_engaged(self)) ||
            (self->mode == GYRO_MODE_ALWAYS_ON);
    static bool active_prev = false;


   

    float x = 0;
    float y = 0;
    float z = 0;
    static float x_prev = 0;
    static float y_prev = 0;
    static float z_prev = 0;
    float dx = 0;
    float dy = 0;
    float dz = 0;
    static float sub_x = 0;
    static float sub_y = 0;
    static float sub_z = 0;
    static float pitch_ref = 0;
    static float roll_ref = 0;

    // Extra protection against jitter.
    if (!active)
    {
        rotation_ref = rotation_state;
        pitch_ref = atan2f(rotation_ref.uy,sqrtf(rotation_ref.uz*rotation_ref.uz + rotation_ref.ux*rotation_ref.ux));
        roll_ref = atan2f(rotation_ref.ux,rotation_ref.uz);
    }
    else
    {
        
        x = -sens_x * (rotation_state.phi - rotation_ref.phi);
        y = sens_y * (atan2f(rotation_state.uy,sqrtf(rotation_state.uz*rotation_state.uz + rotation_state.ux*rotation_state.ux)) - pitch_ref);
        z = sens_z * (atan2f(rotation_state.ux,rotation_state.uz) - roll_ref);

        if(active_prev)
        {
            dx = x-x_prev;
            dy = y-y_prev;
            dz = z-z_prev;
        }
        else
        {
            dx = 0;
            dy = 0;
            dz = 0;
        }
        x_prev = x;
        y_prev = y;
        z_prev = z;
    }

    dx += sub_x;
    dy += sub_y;
    dz += sub_z;
    // Round down and save leftovers.
    sub_x = modff(dx, &x);
    sub_y = modff(dy, &y);
    sub_z = modff(dz, &z);
    
    if (x >= 0) gyro_incremental_output( dx, self->actions_x_pos);
    else        gyro_incremental_output(-dx, self->actions_x_neg);
    if (y >= 0) gyro_incremental_output( dy, self->actions_y_pos);
    else        gyro_incremental_output(-dy, self->actions_y_neg);
    if (z >= 0) gyro_incremental_output( dz, self->actions_z_pos);
    else        gyro_incremental_output(-dz, self->actions_z_neg);
    
    active_prev = active;
}

bool Gyro__is_engaged(Gyro *self) {
    if (self->engage == PIN_NONE) return false;
    if (self->engage == PIN_TOUCH_IN) return touch_status();
    return self->engage_button.is_pressed(&(self->engage_button));
}

void Gyro__report(Gyro *self) {
    Gyro_update_world();
    Gyro_check_offset();

    
    if (self->mode == GYRO_MODE_OFF) {
        return;
    }
    else if (self->mode == GYRO_MODE_AXIS_ABSOLUTE) {
        self->report_absolute(self);
    }
    else
    {
        self->report_incremental(self);
    }
}

void Gyro__reset(Gyro *self) {
    world_init = 0;
    self->pressed_x_pos = false;
    self->pressed_y_pos = false;
    self->pressed_z_pos = false;
    self->pressed_x_neg = false;
    self->pressed_y_neg = false;
    self->pressed_z_neg = false;
}

void Gyro__config_x(Gyro *self, float min, float max, Actions neg, Actions pos) {
    self->absolute_x_min = min;
    self->absolute_x_max = max;
    memcpy(self->actions_x_neg, neg, ACTIONS_LEN);
    memcpy(self->actions_x_pos, pos, ACTIONS_LEN);
}

void Gyro__config_y(Gyro *self, float min, float max, Actions neg, Actions pos) {
    self->absolute_y_min = min;
    self->absolute_y_max = max;
    memcpy(self->actions_y_neg, neg, ACTIONS_LEN);
    memcpy(self->actions_y_pos, pos, ACTIONS_LEN);
}

void Gyro__config_z(Gyro *self, float min, float max, Actions neg, Actions pos) {
    self->absolute_z_min = min;
    self->absolute_z_max = max;
    memcpy(self->actions_z_neg, neg, ACTIONS_LEN);
    memcpy(self->actions_z_pos, pos, ACTIONS_LEN);
}

Gyro Gyro_ (
    GyroMode mode,
    uint8_t engage
) {
    Gyro gyro;
    gyro.is_engaged = Gyro__is_engaged;
    gyro.report = Gyro__report;
    gyro.report_incremental = Gyro__report_incremental_rot_based;
    gyro.report_absolute = Gyro__report_absolute_fast;
    gyro.reset = Gyro__reset;
    gyro.config_x = Gyro__config_x;
    gyro.config_y = Gyro__config_y;
    gyro.config_z = Gyro__config_z;
    gyro.mode = mode;
    gyro.engage = engage;
    if (engage != PIN_NONE && engage != PIN_TOUCH_IN) {
        Actions none = {0,};
        gyro.engage_button = Button_(engage, NORMAL, none, none, none);
    }
    memset(gyro.actions_x_pos, 0, ACTIONS_LEN);
    memset(gyro.actions_y_pos, 0, ACTIONS_LEN);
    memset(gyro.actions_z_pos, 0, ACTIONS_LEN);
    memset(gyro.actions_x_neg, 0, ACTIONS_LEN);
    memset(gyro.actions_y_neg, 0, ACTIONS_LEN);
    memset(gyro.actions_z_neg, 0, ACTIONS_LEN);
    gyro_update_sensitivity();
    gyro.reset(&gyro);
    return gyro;
}
