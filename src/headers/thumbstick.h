// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2022, Input Labs Oy.

#pragma once
#include "button.h"
#include "glyph.h"

#define THUMBSTICK_BASELINE_SATURATION 1.65
#define THUMBSTICK_ADDITIONAL_DEADZONE_FOR_BUTTONS 0.05

#define TS_ROTATION_SMOOTH_SPEED 30
#define TS_ROTATION_SMOOTH_ANGLE 1.0

typedef enum ThumbstickMode_enum {
    THUMBSTICK_MODE_OFF,
    THUMBSTICK_MODE_4DIR,
    THUMBSTICK_MODE_ALPHANUMERIC,
    THUMBSTICK_MODE_8DIR,
    THUMBSTICK_MODE_ROTATION,
} ThumbstickMode;

typedef struct ThumbstickPosition_struct {
    float x;
    float y;
    float angle;
    float radius;
} ThumbstickPosition;

typedef enum Dir4Mask_enum {
    DIR4_MASK_LEFT = 1,
    DIR4_MASK_RIGHT = 2,
    DIR4_MASK_UP = 4,
    DIR4_MASK_DOWN = 8,
} Dir4Mask;

typedef enum Dir4_enum {
    DIR4_NONE,
    DIR4_LEFT,
    DIR4_RIGHT,
    DIR4_UP,
    DIR4_DOWN,
} Dir4;

typedef enum Dir8_enum {
    DIR8_CENTER,
    DIR8_LEFT,
    DIR8_RIGHT,
    DIR8_UP,
    DIR8_DOWN,
    DIR8_UP_LEFT,
    DIR8_UP_RIGHT,
    DIR8_DOWN_LEFT,
    DIR8_DOWN_RIGHT,
} Dir8;

typedef struct _RotationState {
    float angle_smooth;
    float delta_smooth;
    float entry_angle;
    float last_angle;
    float last_value;
    float tracked_angle;
    float tracked_value;
    uint8_t action;
    uint8_t last_action;
    bool has_action;
    bool did_flick;
    float flick_angle;
    float flick_action;
    float flick_time_factor;
} RotationState;

typedef struct Thumbstick_struct Thumbstick;
struct Thumbstick_struct {
    void (*report) (Thumbstick *self);
    void (*report_4dir) (Thumbstick *self, ThumbstickPosition pos, float raw_radius);
    void (*report_4dir_dir) (Thumbstick *self, Button *direction, float value);
    void (*report_4dir_axis) (Thumbstick *self, uint8_t axis, float value);
    void (*report_8dir) (Thumbstick *self, ThumbstickPosition pos, float raw_radius);
    void (*report_rotation) (Thumbstick *self, ThumbstickPosition pos, float raw_radius);
    void (*report_push_auto_toggle) (Thumbstick *self, ThumbstickPosition pos);
    void (*report_alphanumeric) (Thumbstick *self, ThumbstickPosition pos);
    void (*report_glyphstick) (Thumbstick *self, Glyph input);
    void (*report_daisywheel) (Thumbstick *self, Dir8 dir);
    void (*reset) (Thumbstick *self);
    void (*config_4dir) (Thumbstick *self, Button left, Button right, Button up, Button down, Button push, Button inner, Button outer);
    void (*config_8dir) (Thumbstick *self, Button left, Button right, Button up, Button down, Button ul, Button ur, Button dl, Button dr, Button push);
    void (*config_glyphstick) (Thumbstick *self, Actions actions, Glyph glyph);
    void (*config_daisywheel) (Thumbstick *self, uint8_t dir, uint8_t button, Actions actions);
    uint8_t index;
    uint8_t pin_x;
    uint8_t pin_y;
    bool invert_x;
    bool invert_y;
    ThumbstickMode mode;
    bool radial_mode;
    bool deadzone_override;
    float deadzone;
    float antideadzone;
    float overlap;
    float saturation;
    float outer_threshold;
    bool push_auto_toggle;
    float sens_mouse;
    float sens_scroll;
    float sens_xy_ratio;
    float accel_curve;
    float rot_center_deadzone;
    float rot_entry_deadzone;
    bool rot_anticlockwise;
    bool rot_any_angle;
    bool rot_rws_enabled;
    float rot_rws;
    float rot_sens_axis;
    float rot_smoothing;
    float rot_flick_time;
    bool rot_keep_value;
    RotationState rot;
    Button left;
    Button right;
    Button up;
    Button down;
    Button ul;
    Button ur;
    Button dl;
    Button dr;
    Button push;
    Button inner;
    Button outer;
    Glyph glyphstick_glyphs[44];
    Actions glyphstick_actions[44];
    uint8_t glyphstick_index;
    Actions daisywheel[8][4];
};

Thumbstick Thumbstick_ (
    uint8_t index,
    uint8_t pin_x,
    uint8_t pin_y,
    bool invert_x,
    bool invert_y,
    ThumbstickMode mode,
    bool radial_mode,
    bool deadzone_override,
    float deadzone,
    float antideadzone,
    float overlap,
    float saturation,
    float outer_threshold,
    bool push_auto_toggle,
    float sens_mouse,
    float sens_scroll,
    float sens_xy_ratio,
    float accel_curve,
    float rot_center_deadzone,
    float rot_entry_deadzone,
    bool rot_anticlockwise,
    bool rot_any_angle,
    bool rot_rws_enabled,
    float rot_rws,
    float rot_sens_axis,
    float rot_smoothing,
    float rot_flick_time,
    bool rot_keep_value
);

void thumbstick_init();
void thumbstick_report();
void thumbstick_calibrate();
void thumbstick_update_deadzone();
void thumbstick_update_smooth_samples();
void thumbstick_from_ctrl(Thumbstick *thumbstick, CtrlProfile *ctrl, uint8_t index);
uint8_t thumbstick_get_direction(float angle, float overlap);

// thumbstick/dir4.c
void Thumbstick__config_4dir(Thumbstick *self, Button left, Button right, Button up, Button down, Button push, Button inner, Button outer);
void Thumbstick__report_4dir(Thumbstick *self, ThumbstickPosition pos, float raw_radius);
void Thumbstick__report_4dir_dir(Thumbstick *self, Button *direction, float value);
void Thumbstick__report_4dir_axis(Thumbstick *self, uint8_t axis, float value);
void Thumbstick__report_push_auto_toggle(Thumbstick *self, ThumbstickPosition pos);

// thumbstick/dir4.c
void Thumbstick__config_8dir(Thumbstick *self, Button left, Button right, Button up, Button down, Button ul, Button ur, Button dl, Button dr, Button push);
void Thumbstick__report_8dir(Thumbstick *self, ThumbstickPosition pos, float raw_radius);

// thumbstick/gliphstick.c
void thumbstick_init_daisywheel();
void Thumbstick__config_glyphstick(Thumbstick *self, Actions actions, Glyph glyph);
void Thumbstick__config_daisywheel(Thumbstick *self, uint8_t dir, uint8_t button, Actions actions);
void Thumbstick__report_glyphstick(Thumbstick *self, Glyph input);
void Thumbstick__report_daisywheel(Thumbstick *self, Dir8 dir);
void Thumbstick__report_alphanumeric(Thumbstick *self, ThumbstickPosition pos);

// thumbstick/rotation.c
void Thumbstick__report_rotation(Thumbstick *self, ThumbstickPosition pos, float raw_radius);
