// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2022, Input Labs Oy.

/*
This file contains the logic for the thumbstick 4-directional mode.
*/

#include <stdio.h>
#include "thumbstick.h"
#include "config.h"
#include "button.h"
#include "hid.h"

// TODO: Refactor as inmemory struct instead of static.
bool push_toggle_ready = false;
bool push_toggle_engaged = false;

void Thumbstick__config_4dir(
    Thumbstick *self,
    Button left,
    Button right,
    Button up,
    Button down,
    Button push,
    Button inner,
    Button outer
) {
    self->left = left;
    self->right = right;
    self->up = up;
    self->down = down;
    self->push = push;
    self->inner = inner;
    self->outer = outer;
}

void Thumbstick__report_4dir(
    Thumbstick *self,
    ThumbstickPosition pos,
    float raw_radius
) {
    // Evaluate virtual buttons.
    if (pos.radius > THUMBSTICK_ADDITIONAL_DEADZONE_FOR_BUTTONS) {
        if (raw_radius < self->outer_threshold) self->inner.virtual_press = true;
        else self->outer.virtual_press = true;
        uint8_t direction = thumbstick_get_direction(pos.angle, self->overlap);
        if (direction & DIR4_MASK_LEFT)  self->left.virtual_press = true;
        if (direction & DIR4_MASK_RIGHT) self->right.virtual_press = true;
        if (direction & DIR4_MASK_UP)    self->up.virtual_press = true;
        if (direction & DIR4_MASK_DOWN)  self->down.virtual_press = true;
    }
    // Evaluate and report actions.
    float x = pos.x;
    float y = pos.y;
    if (self->radial_mode) {
        x = (x > 0) ? pos.radius : -pos.radius;
        y = (y > 0) ? pos.radius : -pos.radius;
    }
    self->report_4dir_dir(self, &(self->left), -constrain((x), -1, 0));
    self->report_4dir_dir(self, &(self->right), constrain((x),  0, 1));
    self->report_4dir_dir(self, &(self->up),   -constrain((y), -1, 0));
    self->report_4dir_dir(self, &(self->down),  constrain((y),  0, 1));
    // Report inner and outer.
    self->inner.report(&self->inner);
    self->outer.report(&self->outer);
    // Push auto-toggle.
    if (self->push_auto_toggle) {
        self->report_push_auto_toggle(self, pos);
    }
    // Report push.
    self->push.report(&self->push);
}

void Thumbstick__report_4dir_dir(Thumbstick *self, Button *direction, float value) {
    /*
    For the given direction, iterate over its defined actions and report values,
    either as an axis, or via the (virtual) button report system.
    Note that the button report system must be called only once, since it will
    call 'press_multiple' down the line (for all actions, but ignoring axis).
    */
    bool buttons_already_reported = false;
    for(uint8_t i=0; i<4; i++) {
        uint8_t action = direction->actions[i];
        if (action == KEY_NONE) break;
        if (!hid_is_axis(action)) {
            if (buttons_already_reported) continue;
            direction->report(direction);
            buttons_already_reported = true;
        } else {
            self->report_4dir_axis(self, action, value);
        }
    }
}

void Thumbstick__report_4dir_axis(Thumbstick *self, uint8_t axis, float value) {
    // Gamepad.
    if (hid_is_gamepad_axis(axis)) {
        if      (axis == GAMEPAD_AXIS_LX)     hid_gamepad_axis(LX, value);
        else if (axis == GAMEPAD_AXIS_LY)     hid_gamepad_axis(LY, value);
        else if (axis == GAMEPAD_AXIS_RX)     hid_gamepad_axis(RX, value);
        else if (axis == GAMEPAD_AXIS_RY)     hid_gamepad_axis(RY, value);
        else if (axis == GAMEPAD_AXIS_LX_NEG) hid_gamepad_axis(LX, -value);
        else if (axis == GAMEPAD_AXIS_LY_NEG) hid_gamepad_axis(LY, -value);
        else if (axis == GAMEPAD_AXIS_RX_NEG) hid_gamepad_axis(RX, -value);
        else if (axis == GAMEPAD_AXIS_RY_NEG) hid_gamepad_axis(RY, -value);
        else if (axis == GAMEPAD_AXIS_LZ)     hid_gamepad_axis(LZ, value);
        else if (axis == GAMEPAD_AXIS_RZ)     hid_gamepad_axis(RZ, value);
    }
    // Mouse.
    else if (hid_is_mouse_axis(axis)) {
        float mouse_value = value * self->sens_mouse / CFG_TICK_FREQUENCY;
        if      (axis == MOUSE_X)     hid_mouse_move( mouse_value, 0);
        else if (axis == MOUSE_X_NEG) hid_mouse_move(-mouse_value, 0);
        else if (axis == MOUSE_Y)     hid_mouse_move(0,  mouse_value);
        else if (axis == MOUSE_Y_NEG) hid_mouse_move(0, -mouse_value);
    }
    // Scroll.
    else if (hid_is_scroll_axis(axis)) {
        float scroll_value = value * self->sens_scroll / CFG_TICK_FREQUENCY;
        if      (axis == MOUSE_SCROLL_UP)   hid_mouse_scroll(0,  scroll_value);
        else if (axis == MOUSE_SCROLL_DOWN) hid_mouse_scroll(0, -scroll_value);
    }
}

void Thumbstick__report_push_auto_toggle(Thumbstick *self, ThumbstickPosition pos) {
    /*
    CENTER -> EDGE -+-> PUSH (ready) -> RELEASE (^engaged) -+-> CENTER (disengaged)
                    ^                                       |
                    |             [push again]              |
                    +---------------------------------------+

    "Ready" in this context means:
    "button is ready to be toggled at next button release".
    */
    bool is_pressed = self->push.is_pressed(&self->push);
    // Conditions.
    bool condition_ready = (
        !push_toggle_ready &&
        pos.radius >= self->outer_threshold &&
        is_pressed
    );
    bool condition_engage = (
        push_toggle_ready &&
        !push_toggle_engaged &&
        !is_pressed
    );
    bool condition_disengage_press = (
        push_toggle_ready &&
        push_toggle_engaged &&
        !is_pressed
    );
    bool condition_disengage_center = (
        push_toggle_engaged &&
        pos.radius < self->outer_threshold &&
        !is_pressed
    );
    // Logic.
    if (condition_ready) push_toggle_ready = true;
    if (condition_engage) {
        push_toggle_engaged = true;
        push_toggle_ready = false;
    }
    if (condition_disengage_press) {
        push_toggle_engaged = false;
        push_toggle_ready = false;
    }
    if (condition_disengage_center) push_toggle_engaged = false;
    // Fake press.
    if (push_toggle_engaged) self->push.virtual_press = true;
}
