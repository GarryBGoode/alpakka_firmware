// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2022, Input Labs Oy.

/*
This file contains the logic for the thumbstick 8-directional mode.
*/

#include <stdio.h>
#include "thumbstick.h"
#include "config.h"
#include "button.h"
#include "hid.h"

void Thumbstick__config_8dir(
    Thumbstick *self,
    Button left,
    Button right,
    Button up,
    Button down,
    Button ul,
    Button ur,
    Button dl,
    Button dr,
    Button push
) {
    self->left = left;
    self->right = right;
    self->up = up;
    self->down = down;
    self->ul = ul;
    self->ur = ur;
    self->dl = dl;
    self->dr = dr;
    self->push = push;
}

void Thumbstick__report_8dir(
    Thumbstick *self,
    ThumbstickPosition pos,
    float raw_radius
) {
    // Evaluate virtual buttons.
    if (raw_radius > self->deadzone + THUMBSTICK_ADDITIONAL_DEADZONE_FOR_BUTTONS) {
        uint8_t direction = thumbstick_get_direction(pos.angle, 0.5); // Fixed overlap.
        if      (direction == DIR4_MASK_LEFT)  self->left.virtual_press = true;
        else if (direction == DIR4_MASK_RIGHT) self->right.virtual_press = true;
        else if (direction == DIR4_MASK_UP)    self->up.virtual_press = true;
        else if (direction == DIR4_MASK_DOWN)  self->down.virtual_press = true;
        else if (direction == (DIR4_MASK_UP   + DIR4_MASK_LEFT))  self->ul.virtual_press = true;
        else if (direction == (DIR4_MASK_UP   + DIR4_MASK_RIGHT)) self->ur.virtual_press = true;
        else if (direction == (DIR4_MASK_DOWN + DIR4_MASK_LEFT))  self->dl.virtual_press = true;
        else if (direction == (DIR4_MASK_DOWN + DIR4_MASK_RIGHT)) self->dr.virtual_press = true;
    }
    // Report directional virtual buttons.
    self->left.report(&self->left);
    self->right.report(&self->right);
    self->up.report(&self->up);
    self->down.report(&self->down);
    self->ul.report(&self->ul);
    self->ur.report(&self->ur);
    self->dl.report(&self->dl);
    self->dr.report(&self->dr);
    // Report push.
    self->push.report(&self->push);
}
