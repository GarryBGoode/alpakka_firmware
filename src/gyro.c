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
#include "led.h"
#include "pin.h"
#include "touch.h"
#include "vector.h"
#include "rotation_ekf.h"
#include "rotation_fast.h"

double sensitivity_multiplier;


uint8_t world_init = 0;
Vector world_top;
Vector world_fw;
Vector world_right;
Vector accel_smooth;
Vector imu_gyro_global;
Vector imu_accel_global;

float z_int = 0.0f;
float z_int_threshold = 50.0f;

FloatVector gyro_integral = {0, 0, 0};
FloatVector gyro_ref_point = {0, 0, 0};
IntVector gyro_val_rounded = {0, 0, 0};

EKF main_ekf;

RotationStateVector main_rotvec = {
    .ux = 0.0f,
    .uy = 0.0f,
    .uz = 1.0f, // Default to Z-axis up.
    .phi = 0.0f
};

RotationStateVector ref_rotvec = {
    .ux = 0.0f,
    .uy = 0.0f,
    .uz = 1.0f, // Default to Z-axis up.
    .phi = 0.0f
};

float ekf_q0[4] = {1, 0, 0, 0};
float ekf_b0[3] = {0, 0, 0};
float gyro_bias_err_P0 = 0.0f; // Initial gyro bias error.
float gyro_noise = 0.01f; // Gyro noise.
float gyro_bias_noise = 0.00001f; // Gyro bias noise.
float accelerometer_noise = 1.0f; // Accelerometer noise.


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

void gyro_incremental_output(int16_t value, uint8_t *actions) {
    for(uint8_t i=0; i<4; i++) {
        uint8_t action = actions[i];
        if      (action == MOUSE_X)     hid_mouse_move(value, 0);
        else if (action == MOUSE_Y)     hid_mouse_move(0, value);
        else if (action == MOUSE_X_NEG) hid_mouse_move(-value, 0);
        else if (action == MOUSE_Y_NEG) hid_mouse_move(0, -value);
    }
}

double hssnf(double t, double k, double x) {
    //t = 1, k = 0.5
    double a = x - (x * k);
    double b = 1 - (x * k * (1/t));
    return a / b;
}

void Gyro__report_absolute(Gyro *self) {
    // Accel-based correction.
    gyro_accel_correction();
    // Get data from gyros.
    Vector gyro = imu_read_gyro();
    static float sens = -BIT_18 * M_PI;
    // Rotate world space orientation.
    Vector4 rx = quaternion(world_right, gyro.y / sens);
    Vector4 ry = quaternion(world_fw, gyro.z / sens);
    Vector4 rz = quaternion(world_top, gyro.x / sens);
    static uint8_t i = 0;
    Vector4 r;
    if      (i==0) r = qmultiply(qmultiply(rx, ry), rz);
    else if (i==1) r = qmultiply(qmultiply(rz, rx), ry);
    else if (i==2) r = qmultiply(qmultiply(ry, rz), rx);
    else if (i==3) r = qmultiply(qmultiply(rx, rz), ry);
    else if (i==4) r = qmultiply(qmultiply(ry, rx), rz);
    else if (i==5) r = qmultiply(qmultiply(rz, ry), rx);
    i++;
    if (i>5) i = 0;
    world_top = qrotate(r, world_top);
    world_fw = qrotate(r, world_fw);
    world_right = vector_cross_product(world_fw, world_top);
    // Debug.
    bool debug = 0;
    if (debug) {
        hid_gamepad_axis(LX, world_top.x);
        hid_gamepad_axis(LY, -world_top.y);
        hid_gamepad_axis(RX, world_fw.x);
        hid_gamepad_axis(RY, -world_fw.y);
        return;
    }
    // Output calculation.
    float x = degrees(asin(-world_right.z)) / 90;
    float y = degrees(asin(-world_top.z)) / 90;
    float z = degrees(asin(world_fw.z)) / 90;
    if (fabs(x) > 0.5 && z < 0) x += -z * 2 * sign(x); // Steering lock.
    x = constrain(x * 1.1, -1, 1); // Additional saturation.
    x = ramp(x, self->absolute_x_min/90, self->absolute_x_max/90); // Adjust range.
    y = ramp(y, self->absolute_y_min/90, self->absolute_y_max/90); // Adjust range.
    // Output mapping.
    if (x >= 0) gyro_absolute_output( x, self->actions_x_pos, &(self->pressed_x_pos));
    else        gyro_absolute_output(-x, self->actions_x_neg, &(self->pressed_x_neg));
    if (y >= 0) gyro_absolute_output( y, self->actions_y_pos, &(self->pressed_y_pos));
    else        gyro_absolute_output(-y, self->actions_y_neg, &(self->pressed_y_neg));
    // printf("\r%6.1f %6.1f %6.1f", x*100, y*100, z*100);
}

void Gyro__report_incremental(Gyro *self, bool engaged) {

    FloatVector gyro;
    FloatVector accel;
    int16_t x, y, z;
    imu_update(&gyro, &accel);
    imu_accel_global.x = accel.x;
    imu_accel_global.y = accel.y;
    imu_accel_global.z = accel.z;
    float gyro_abs = 1.0f;

    /* This deadzone is a bit of a hack, it helps reduce noise, feels similar to friction */
    if (CFG_GYRO_DEADZONE_CONSTANT>0){
        gyro_abs=    CFG_GYRO_PIX_SENSITIVITY / CFG_TICK_FREQUENCY / CFG_GYRO_DEADZONE_CONSTANT * sensitivity_multiplier *
                     CFG_GYRO_PIX_SENSITIVITY / CFG_TICK_FREQUENCY / CFG_GYRO_DEADZONE_CONSTANT * sensitivity_multiplier *
                    (gyro.x * gyro.x +
                     gyro.y * gyro.y +
                     gyro.z * gyro.z);
    }
    float deadzone_multiplier = 1.0f;
    if (gyro_abs<1.0f)
    {
        /* gyro abs is a length-square type value */
        /* sqrt of that makes it closer to linear */
        deadzone_multiplier = (sqrtf(gyro_abs));
    }
    /* Integrating gyro signal */
    /* Note: integration is done in float, noise drift is inevitable without deadzone*/
    gyro_integral.x += gyro.x/CFG_TICK_FREQUENCY*deadzone_multiplier;
    gyro_integral.y += gyro.y/CFG_TICK_FREQUENCY*deadzone_multiplier;
    gyro_integral.z += gyro.z/CFG_TICK_FREQUENCY*deadzone_multiplier;
    float w_vector[3] = {gyro.y*deadzone_multiplier, gyro.z*deadzone_multiplier, gyro.x*deadzone_multiplier};
    float accel_vector[3] = {accel.x, accel.y, accel.z};

    update_rotation_state(&main_rotvec, w_vector, accel_vector, 1.0f/CFG_TICK_FREQUENCY);
    // EKF_predict(&main_ekf, w_vector, 1.0f/CFG_TICK_FREQUENCY);
    // EKF_update(&main_ekf, accel_vector);

    /* Ratcheting logic:
       There's a reference point, the mouse is considered to move relative to it.
       When not engaged, ref point moves with gyro integral, so no movement.
       When engaged, gyro integral moves relative to ref point, mouse movement applies.
       Ultimately, mouse is reported as increments in int16 type.
       A direct conversion gyro-to-int can cause an undesired deadzone.
       Integrating in float without deadzone causes noise drift. 
       Rounding and differentiating logic is added here to work around this.
       Soft deadzone is calculated above.
       */
    x= CFG_GYRO_PIX_SENSITIVITY*CFG_GYRO_SENSITIVITY_X*sensitivity_multiplier*(main_rotvec.phi- ref_rotvec.phi);
    y= CFG_GYRO_PIX_SENSITIVITY*CFG_GYRO_SENSITIVITY_Y*sensitivity_multiplier*(main_rotvec.uy - ref_rotvec.uy);
    z= CFG_GYRO_PIX_SENSITIVITY*CFG_GYRO_SENSITIVITY_Z*sensitivity_multiplier*(main_rotvec.ux - ref_rotvec.ux);
    
    float z_delta=0.0f;
    

    if (engaged)
    {
        if(main_rotvec.ux>0.15f){
            z_delta = (main_rotvec.ux-0.15f)/main_rotvec.uz;
            if (z_delta >1) z_delta = 1;
        }
        else if(main_rotvec.ux<-0.15f){
            z_delta = (main_rotvec.ux+0.15f)/main_rotvec.uz;;
            if (z_delta < -1) z_delta = -1;
        }
        else {
            z_delta = 0.0f;
        }

        z_int += (z_delta) * CFG_GYRO_PIX_SENSITIVITY * CFG_GYRO_SENSITIVITY_Z * sensitivity_multiplier/ CFG_TICK_FREQUENCY;
        z = (int16_t)z_int;

        if(gyro_val_rounded.x!=x)
        {
            int16_t x_diff = x - gyro_val_rounded.x;
            if (x_diff >= 0) gyro_incremental_output( x_diff, self->actions_x_pos);
            else gyro_incremental_output(-x_diff, self->actions_x_neg);
            gyro_val_rounded.x = x;
        }
        if(gyro_val_rounded.y != y)
        {
            int16_t y_diff = y - gyro_val_rounded.y;
            if (y_diff >= 0) gyro_incremental_output( y_diff, self->actions_y_pos);
            else gyro_incremental_output(-y_diff, self->actions_y_neg);
            gyro_val_rounded.y = y;
        }
        if(gyro_val_rounded.z != z)
        {
            int16_t z_diff = z - gyro_val_rounded.z;
            if (z_diff >= 0) gyro_incremental_output( z_diff, self->actions_z_pos);
            else gyro_incremental_output(-z_diff, self->actions_z_neg);
            gyro_val_rounded.z = z;
        }
            

    }
    else {
        ref_rotvec.phi = main_rotvec.phi;
        ref_rotvec.ux = main_rotvec.ux;
        ref_rotvec.uy = main_rotvec.uy;
        ref_rotvec.uz = main_rotvec.uz;
        gyro_val_rounded.x= 0;
        gyro_val_rounded.y= 0;
        gyro_val_rounded.z= 0;
        z_int = 0.0f; // Reset the integral.
    }
}

bool Gyro__is_engaged(Gyro *self) {
    if (self->engage == PIN_NONE) return false;
    if (self->engage == PIN_TOUCH_IN) return touch_status();
    return self->engage_button.is_pressed(&(self->engage_button));
}

void Gyro__report(Gyro *self) {
    if (self->mode == GYRO_MODE_TOUCH_ON) {
        self->report_incremental(self,self->is_engaged(self));
    }
    else if (self->mode == GYRO_MODE_TOUCH_OFF) {
        self->report_incremental(self, !self->is_engaged(self));
    }
    else if (self->mode == GYRO_MODE_ALWAYS_ON) {
        self->report_incremental(self,true);
    }
    else if (self->mode == GYRO_MODE_AXIS_ABSOLUTE) {
        self->report_absolute(self);
    }
    else if (self->mode == GYRO_MODE_OFF) {
        return;
    }
}

void Gyro__reset(Gyro *self) {
    world_init = 0;
    EKF_init(&main_ekf, ekf_q0, ekf_b0, gyro_bias_err_P0, gyro_noise, gyro_bias_noise, accelerometer_noise);
    main_rotvec.ux = 0.0f;
    main_rotvec.uy = 0.0f;
    main_rotvec.uz = 1.0f; // Default to Z-axis up.
    main_rotvec.phi = 0.0f;
    self->pressed_x_pos = false;
    self->pressed_y_pos = false;
    self->pressed_z_pos = false;
    self->pressed_x_neg = false;
    self->pressed_y_neg = false;
    self->pressed_z_neg = false;
}

void Gyro__config_x(Gyro *self, double min, double max, Actions neg, Actions pos) {
    self->absolute_x_min = min;
    self->absolute_x_max = max;
    memcpy(self->actions_x_neg, neg, ACTIONS_LEN);
    memcpy(self->actions_x_pos, pos, ACTIONS_LEN);
}

void Gyro__config_y(Gyro *self, double min, double max, Actions neg, Actions pos) {
    self->absolute_y_min = min;
    self->absolute_y_max = max;
    memcpy(self->actions_y_neg, neg, ACTIONS_LEN);
    memcpy(self->actions_y_pos, pos, ACTIONS_LEN);
}

void Gyro__config_z(Gyro *self, double min, double max, Actions neg, Actions pos) {
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
    gyro.report_incremental = Gyro__report_incremental;
    gyro.report_absolute = Gyro__report_absolute;
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
