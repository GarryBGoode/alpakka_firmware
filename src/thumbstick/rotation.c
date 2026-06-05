// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2022, Input Labs Oy.

/*
This file contains the logic for the thumbstick rotation mode.
*/

#include "thumbstick.h"
#include "config.h"
#include "hid.h"

static void reset_state(Thumbstick *ts) {
    ts->rot.angle_smooth = 0;
    ts->rot.delta_smooth = 0;
    ts->rot.action = KEY_NONE;
    ts->rot.has_action = false;
    ts->rot.did_flick = false;
    if (!ts->rot_any_angle) {
        ts->rot.tracked_angle = 0;
        ts->rot.tracked_value = 0;
    }
}

static void find_action(Thumbstick *ts, ThumbstickPosition pos) {
    /*
    Find the action and entry angle, by walking (clockwise or anticlockwise)
    from the initial angle to the first cardinal with a defined action.
    Also configure other perimeter-related settings.
    */
    uint8_t actions[4] = {ts->up.actions[0], ts->right.actions[0], ts->down.actions[0], ts->left.actions[0]};
    // Determine entry quarter.
    float angle_360 = pos.angle > 0 ? pos.angle : pos.angle+360;  // Reframe -180:180 to 0:360.
    uint8_t quarter = floorf(angle_360 / 90);  // Get initial angle quarter 0|1|2|3.
    // Walk in circle.
    for(uint8_t i=0; i<4; i++) {
        int8_t ii = ts->rot_anticlockwise ? (int8_t)i : (int8_t)-i;
        uint8_t offset = (quarter+4 + ii + (ts->rot_anticlockwise ? 1 : 0)) % 4;
        if (actions[offset]) {
            ts->rot.action = actions[offset];
            ts->rot.has_action = true;
            ts->rot.entry_angle = offset * 90;  // Cardinal entry angle of defined action.
            if (ts->rot.entry_angle >  180) ts->rot.entry_angle -= 360;  // Reframe 0:360 to -180:180.
            if (ts->rot.entry_angle < -180) ts->rot.entry_angle += 360;  // Reframe 0:360 to -180:180.
            ts->rot.last_angle = ts->rot_any_angle ? pos.angle : ts->rot.entry_angle;
            break;
        }
    }
}

static void report_mouse(Thumbstick *ts, float delta_angle, uint8_t action, bool is_in_deadzone, bool can_be_delayed) {
    // Deadzone.
    if (is_in_deadzone && !ts->rot.did_flick) {
        ts->rot.last_angle = ts->rot.angle_smooth;
        ts->rot.did_flick = true;
        return;
    }
    // Flick-time delay.
    if (can_be_delayed) {
        ts->rot.flick_angle += delta_angle;
        ts->rot.flick_action = action;
        ts->rot.did_flick = true;
        return;
    }
    // Calculate sensitivity.
    float pixels_per_degree;
    if (ts->rot_rws_enabled) {
        float gyro_sens = config_get_mouse_sens_value(config_get_mouse_sens_preset());
        pixels_per_degree = ((gyro_sens * 1920) / 45) / ts->rot_rws;
    } else {
        pixels_per_degree = ts->sens_mouse / 360.0f;
    }
    // Report.
    float value = delta_angle * pixels_per_degree;
    if      (action == MOUSE_X)     hid_mouse_move( value, 0);
    else if (action == MOUSE_X_NEG) hid_mouse_move(-value, 0);
    else if (action == MOUSE_Y)     hid_mouse_move(0,  value);
    else if (action == MOUSE_Y_NEG) hid_mouse_move(0, -value);
}

static void report_scroll(Thumbstick *ts, float delta_angle, uint8_t action) {
    // Calculate sensitivity.
    float ticks_per_degree = ts->sens_scroll / 360.0f;
    // Report.
    float value = delta_angle * ticks_per_degree;
    if      (action == MOUSE_SCROLL_UP)   hid_mouse_scroll(0,  value);
    else if (action == MOUSE_SCROLL_DOWN) hid_mouse_scroll(0, -value);
}

static void report_gamepad(Thumbstick *ts, float delta_angle, uint8_t action, bool is_in_deadzone) {
    ts->rot.tracked_angle += delta_angle;
    ts->rot.tracked_angle = wrap_angle_360(ts->rot.tracked_angle);
    float sens = ts->rot_sens_axis / 360.0f;
    float value = (
        ts->rot_any_angle ?
        ts->rot.tracked_value + (delta_angle * sens) :  // Relative.
        ts->rot.tracked_angle * sens                    // Absolute.
    );
    bool axis_is_signed = (action != GAMEPAD_AXIS_LZ) && (action != GAMEPAD_AXIS_RZ);
    value = constrain(value, (axis_is_signed ? -1.0f : 0.0f), 1.0f);
    if (is_in_deadzone && !ts->rot_any_angle) value = 0;
    ts->rot.tracked_value = value;
    ts->rot.last_value = value;
    ts->rot.last_action = action;
    bool axis_is_positive = action <= GAMEPAD_AXIS_RZ;  // Last positive gamepad axis.
    GamepadAxis gamepad_axis = action - GAMEPAD_AXIS_INDEX - (axis_is_positive ? 0 : 6);  // GamepadAxis has no negative variants.
    float send_value = axis_is_positive ? value : -value;
    hid_gamepad_axis(gamepad_axis, send_value);
    // info("G %.2f\n", send_value);
}

static void report_gamepad_resend(Thumbstick *ts) {
    /*
    Since HID gamepad logic aggregates multiple values from separate inputs, it
    reset the value to zero in each tick. This is convenient for most use cases,
    except the corner case of the rotation mode "holding" the value while is at
    the stick is at the center (keep value setting). So we have to resend the
    value each tick.
    Note that resending values to the HID layer does not imply data is send via
    USB all ticks.
    */
    uint8_t action = ts->rot.last_action - GAMEPAD_AXIS_INDEX;
    hid_gamepad_axis(action, ts->rot.last_value);
}

static void calc_flick_time(Thumbstick *ts) {
    /*
    Calculate the factor for a max flick (180 degrees) to exponentially decay
    into less than 1 degree, in the target time, with the current polling rate.
    */
    float base = 1.0f / 180.0f;
    float exp = 1.0f / (ts->rot_flick_time / CFG_TICK_INTERVAL_IN_MS);
    ts->rot.flick_time_factor = 1 - pow(base, exp);
}

static void apply_flick_time(Thumbstick *ts) {
    if (fabsf(ts->rot.flick_angle) < 0.1f) return;
    float factor = ts->rot.flick_time_factor;
    float delta_angle = ts->rot.flick_angle * factor;
    ts->rot.flick_angle *= 1-factor;
    report_mouse(ts, delta_angle, ts->rot.flick_action, false, false);
}

void Thumbstick__report_rotation(Thumbstick *self, ThumbstickPosition pos, float raw_radius) {
    /*
    Thumbstick method for reporting rotation mode, extracted to this file.
    */
    // Push button.
    self->push.report(&self->push);
    // Smoothed flick (if previously set).
    apply_flick_time(self);
    // When back to center.
    if (pos.radius == 0) {
        reset_state(self);
        if (self->rot_keep_value) report_gamepad_resend(self);
        return;
    }
    // When stick moves from center to perimeter.
    if (!self->rot.has_action) {
        find_action(self, pos);
        if (self->rot_flick_time != 0) calc_flick_time(self);
    }
    // Determine angle.
    float angle = pos.angle;
    if (self->rot.angle_smooth == 0) self->rot.angle_smooth = angle;
    if (self->rot.angle_smooth >  90 && angle < -90) angle += 360;  // Shortest path to prev angle.
    if (self->rot.angle_smooth < -90 && angle >  90) angle -= 360;
    // Smoothing.
    float smooth_cutoff = fabsf(self->rot.delta_smooth) / TS_ROTATION_SMOOTH_ANGLE;
    float smooth_factor = interpolate(self->rot_smoothing, 1, smooth_cutoff);
    self->rot.angle_smooth = wrap_angle_180(smooth(self->rot.angle_smooth, angle, smooth_factor));
    // Deadzone.
    bool is_in_deadzone = is_between(
        (self->rot.entry_angle != 180) ? self->rot.angle_smooth : fabs(self->rot.angle_smooth),
        self->rot.entry_angle - self->rot_entry_deadzone / 2,
        self->rot.entry_angle + self->rot_entry_deadzone / 2
    );
    // Delta.
    float delta_angle = wrap_angle_180(self->rot.angle_smooth - self->rot.last_angle);
    if (self->rot_anticlockwise) delta_angle = -delta_angle;
    self->rot.delta_smooth = smooth(self->rot.delta_smooth, delta_angle, TS_ROTATION_SMOOTH_SPEED);
    // Mouse.
    if (hid_is_mouse_axis(self->rot.action)) {
        bool can_be_delayed = !self->rot.did_flick && self->rot_flick_time > 0;
        report_mouse(self, delta_angle, self->rot.action, is_in_deadzone, can_be_delayed);
    }
    // Scroll.
    else if (hid_is_scroll_axis(self->rot.action)) {
        report_scroll(self, delta_angle, self->rot.action);
    }
    // Gamepad.
    else if (hid_is_gamepad_axis(self->rot.action)) {
        report_gamepad(self, delta_angle, self->rot.action, is_in_deadzone);
    }
    // Update last angle.
    self->rot.last_angle = self->rot.angle_smooth;
}
