// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2022, Input Labs Oy.

/*
HID layer is responsible for managing all the outputs that are sent to the
operating system (via USB or wireless), by keeping a register of all the active
actions (requested by the current profile) and syncing its state with the
available interfaces (keyboard, mouse, gamepad...).

Physical controller --> Profile --> [HID layer] --> HID keyboard
                    --> Profile -->             --> HID mouse
                    --> Profile -->             --> XInput gamepad
                    --> Profile -->             --> Internal procedures

At the end of each cycle (determined by the polling rate) the HID layer checks
if the potential new report is different from the last report sent to the
interfaces (USB keyboard, USB mouse, gamepad...), and sends the report if
required.

The state matrix is a representation of all the actions that could be sent
(output) and internal operations (procedures) requested by the user. It keep
track of how many references to these actions are active.
As a simplification: Button presses increase the counter by one, and button
releases decrease the counter by one.

To avoid orphan references, the state matrix is usually re-initialized (reset)
to zeros when the user changes the active profile, otherwise the disabled
profile won't ever trigger the corresponding counter decrease of held buttons
during the profile change.
*/

#include <tusb.h>
#include <device/usbd_pvt.h>
#include "config.h"
#include "ctrl.h"
#include "hid.h"
#include "wireless.h"
#include "led.h"
#include "profile.h"
#include "xinput.h"
#include "common.h"
#include "webusb.h"
#include "logging.h"
#include "thanks.h"
#include "power.h"

bool hid_allow_communication = true;  // Extern.

bool synced_keyboard = false;
bool synced_mouse = false;
bool synced_gamepad = false;
uint8_t priority_mouse = 0;
uint8_t priority_gamepad = 0;

uint16_t alarms = 0;
alarm_pool_t *alarm_pool;

uint8_t state_matrix[256] = {0,};
int16_t mouse_x = 0;
int16_t mouse_y = 0;
double gamepad_axis[6] = {0,};
double gamepad_axis_last[6] = {0,};

void hid_matrix_reset(uint8_t keep) {
    for(uint8_t action=0; action<255; action++) {
        if (action == keep) continue;  // Optionally do not reset specific actions.
        state_matrix[action] = 0;
    }
    synced_keyboard = false;
    synced_mouse = false;
    synced_gamepad = false;
}

void hid_procedure_press(uint8_t procedure){
    if (procedure == PROC_HOME) profile_set_home(true);                  // Hold home.
    if (procedure == PROC_HOME_GAMEPAD) profile_set_home_gamepad(true);  // Double-click-hold home.
    if (procedure == PROC_PROFILE_1) profile_set_active(1);
    if (procedure == PROC_PROFILE_2) profile_set_active(2);
    if (procedure == PROC_PROFILE_3) profile_set_active(3);
    if (procedure == PROC_PROFILE_4) profile_set_active(4);
    if (procedure == PROC_PROFILE_5) profile_set_active(5);
    if (procedure == PROC_PROFILE_6) profile_set_active(6);
    if (procedure == PROC_PROFILE_7) profile_set_active(7);
    if (procedure == PROC_PROFILE_8) profile_set_active(8);
    if (procedure == PROC_PROFILE_9) profile_set_active(9);
    if (procedure == PROC_PROFILE_10) profile_set_active(10);
    if (procedure == PROC_PROFILE_11) profile_set_active(11);
    if (procedure == PROC_PROFILE_12) profile_set_active(12);
    if (procedure == PROC_TUNE_UP) config_tune(1);
    if (procedure == PROC_TUNE_DOWN) config_tune(0);
    if (procedure == PROC_TUNE_OS) config_tune_set_mode(procedure);
    if (procedure == PROC_TUNE_MOUSE_SENS) config_tune_set_mode(procedure);
    if (procedure == PROC_TUNE_TOUCH_SENS) config_tune_set_mode(procedure);
    if (procedure == PROC_TUNE_DEADZONE) config_tune_set_mode(procedure);
    if (procedure == PROC_CALIBRATE) config_calibrate();
    if (procedure == PROC_RESTART) power_restart();
    if (procedure == PROC_BOOTSEL) power_bootsel();  // TODO: BOORSEL_OR_PAIR
    if (procedure == PROC_THANKS) hid_thanks();
    if (procedure == PROC_IGNORE_LED_WARNINGS) config_ignore_problems();
    if (procedure == PROC_SLEEP) power_dormant();
    // Scrollwheel alternative modes. (Used for example in Racing profile).
    if (procedure == PROC_ROTARY_MODE_0) rotary_set_mode(0);
    if (procedure == PROC_ROTARY_MODE_1) rotary_set_mode(1);
    if (procedure == PROC_ROTARY_MODE_2) rotary_set_mode(2);
    if (procedure == PROC_ROTARY_MODE_3) rotary_set_mode(3);
    if (procedure == PROC_ROTARY_MODE_4) rotary_set_mode(4);
    if (procedure == PROC_ROTARY_MODE_5) rotary_set_mode(5);
    // Macros.
    if (procedure == PROC_MACRO_1) hid_macro(1);
    if (procedure == PROC_MACRO_2) hid_macro(2);
    if (procedure == PROC_MACRO_3) hid_macro(3);
    if (procedure == PROC_MACRO_4) hid_macro(4);
    if (procedure == PROC_MACRO_5) hid_macro(5);
    if (procedure == PROC_MACRO_6) hid_macro(6);
    if (procedure == PROC_MACRO_7) hid_macro(7);
    if (procedure == PROC_MACRO_8) hid_macro(8);
}

void hid_procedure_release(uint8_t procedure) {
    if (procedure == PROC_HOME) profile_set_home(false);
    if (procedure == PROC_HOME_GAMEPAD) profile_set_home_gamepad(false);
}

void hid_press(uint8_t key) {
    if (key == KEY_NONE) return;
    else if (key >= PROC_INDEX) hid_procedure_press(key);
    else {
        state_matrix[key] += 1;
        if (key >= GAMEPAD_INDEX) synced_gamepad = false;
        else if (key >= MOUSE_INDEX) synced_mouse = false;
        else synced_keyboard = false;
    }
}

void hid_release(uint8_t key) {
    if (key == KEY_NONE) return;
    else if (key == MOUSE_SCROLL_UP) return;
    else if (key == MOUSE_SCROLL_DOWN) return;
    else if (key >= PROC_INDEX) hid_procedure_release(key);
    else {
        if (state_matrix[key] > 0) {  // Do not allow to wrap / go negative.
            state_matrix[key] -= 1;
            if (key >= GAMEPAD_INDEX) synced_gamepad = false;
            else if (key >= MOUSE_INDEX) synced_mouse = false;
            else synced_keyboard = false;
        }
    }
}

void hid_press_multiple(uint8_t *keys) {
    for(uint8_t i=0; i<ACTIONS_LEN; i++) {
        if (keys[i] == 0) break;
        hid_press(keys[i]);
    }
    if (keys[0] == PROC_HOME) return; // Home does not count as input.
    profile_set_reported_inputs(true);

}

void hid_release_multiple(uint8_t *keys) {
    for(uint8_t i=0; i<ACTIONS_LEN; i++) {
        if (keys[i] == 0) return;
        hid_release(keys[i]);
    }
}

void hid_press_later(uint8_t key, uint16_t delay) {
    alarm_pool_add_alarm_in_ms(
        alarm_pool,
        delay,
        (alarm_callback_t)hid_press_later_callback,
        (void*)(uint32_t)key,
        true
    );
    alarms++;
}

void hid_release_later(uint8_t key, uint16_t delay) {
    alarm_pool_add_alarm_in_ms(
        alarm_pool,
        delay,
        (alarm_callback_t)hid_release_later_callback,
        (void*)(uint32_t)key,
        true
    );
    alarms++;
}

void hid_press_multiple_later(uint8_t *keys, uint16_t delay) {
    alarm_pool_add_alarm_in_ms(
        alarm_pool,
        delay,
        (alarm_callback_t)hid_press_multiple_later_callback,
        keys,
        true
    );
    alarms++;
}

void hid_release_multiple_later(uint8_t *keys, uint16_t delay) {
    alarm_pool_add_alarm_in_ms(
        alarm_pool,
        delay,
        (alarm_callback_t)hid_release_multiple_later_callback,
        keys,
        true
    );
    alarms++;
}

void hid_press_later_callback(alarm_id_t alarm, uint8_t key) {
    alarm_pool_cancel_alarm(alarm_pool, alarm);
    hid_press(key);
    alarms--;
}

void hid_release_later_callback(alarm_id_t alarm, uint8_t key) {
    alarm_pool_cancel_alarm(alarm_pool, alarm);
    hid_release(key);
    alarms--;
}

void hid_press_multiple_later_callback(alarm_id_t alarm, uint8_t *keys) {
    alarm_pool_cancel_alarm(alarm_pool, alarm);
    hid_press_multiple(keys);
    alarms--;
}

void hid_release_multiple_later_callback(alarm_id_t alarm, uint8_t *keys) {
    alarm_pool_cancel_alarm(alarm_pool, alarm);
    hid_release_multiple(keys);
    alarms--;
}

void hid_macro(uint8_t index) {
    uint8_t section = SECTION_MACRO_1 + ((index - 1) / 2);
    uint8_t subindex = (index - 1) % 2;
    CtrlProfile *profile = config_profile_read(profile_get_active_index(false));
    uint8_t *macro = profile->sections[section].macro.macro[subindex];
    if (alarms > 0) return;  // Disallows parallel macros. TODO fix.
    uint16_t time = 10;
    for(uint8_t i=0; i<28; i++) {
        if (macro[i] == 0) break;
        hid_press_later(macro[i], time);
        time += 10;
        hid_release_later(macro[i], time);
        time += 10;
    }
}

bool hid_is_axis(uint8_t key) {
    return is_between(key, GAMEPAD_AXIS_INDEX, PROC_INDEX-1);
}

void hid_mouse_move(int16_t x, int16_t y) {
    mouse_x += x;
    mouse_y += y;
    synced_mouse = false;
    profile_set_reported_inputs(true);
}

void hid_gamepad_axis(GamepadAxis axis, double value) {
    gamepad_axis[axis] += value;  // Multiple inputs can be combined.
    if (value != 0) profile_set_reported_inputs(true);
}

MouseReport hid_get_mouse_report() {
    // Create button bitmask.
    int8_t buttons = 0;
    for(int i=0; i<5; i++) {
        buttons += state_matrix[MOUSE_INDEX + i] << i;
    }
    uint8_t scroll = state_matrix[MOUSE_SCROLL_UP] - state_matrix[MOUSE_SCROLL_DOWN];
    // Create report.
    MouseReport report = {buttons, mouse_x, mouse_y, scroll, 0};
    return report;
}

KeyboardReport hid_get_keyboard_report() {
    // Keys.
    uint8_t keys[6] = {0};
    uint8_t keys_available = 6;
    for(int i=0; i<=115; i++) {
        if (state_matrix[i] >= 1) {
            keys[keys_available - 1] = (uint8_t)i;
            keys_available--;
            if (keys_available == 0) {
                break;
            }
        }
    }
    // Modifiers.
    uint8_t modifiers = 0;
    for(int i=0; i<8; i++) {
        // Any value bigger than 1 consolidates to 1 (with !!).
        modifiers += !!state_matrix[MODIFIER_INDEX + i] << i;
    }
    // Create report.
    KeyboardReport report = {modifiers};
    memcpy(report.keycode, keys, 6);
    return report;
}

double hid_axis(
    double value,
    uint8_t matrix_index_pos,
    uint8_t matrix_index_neg
) {
    if (matrix_index_neg) {
        if (state_matrix[matrix_index_neg]) return -1;
        else if (state_matrix[matrix_index_pos]) return 1;
        else return constrain(value, -1, 1);
    } else {
        if (state_matrix[matrix_index_pos]) return 1;
        else return constrain(fabs(value), 0, 1);
    }
}

GamepadReport hid_get_gamepad_report() {
    // Sorted so the most common assigned buttons are lower and easier to
    // identify in-game.
    int32_t buttons = (
        // Any value bigger than 1 consolidates to 1 (with !!).
        ((!!state_matrix[GAMEPAD_A])      <<  0) +
        ((!!state_matrix[GAMEPAD_B])      <<  1) +
        ((!!state_matrix[GAMEPAD_X])      <<  2) +
        ((!!state_matrix[GAMEPAD_Y])      <<  3) +
        ((!!state_matrix[GAMEPAD_L1])     <<  4) +
        ((!!state_matrix[GAMEPAD_R1])     <<  5) +
        ((!!state_matrix[GAMEPAD_L3])     <<  6) +
        ((!!state_matrix[GAMEPAD_R3])     <<  7) +
        ((!!state_matrix[GAMEPAD_LEFT])   <<  8) +
        ((!!state_matrix[GAMEPAD_RIGHT])  <<  9) +
        ((!!state_matrix[GAMEPAD_UP])     << 10) +
        ((!!state_matrix[GAMEPAD_DOWN])   << 11) +
        ((!!state_matrix[GAMEPAD_SELECT]) << 12) +
        ((!!state_matrix[GAMEPAD_START])  << 13) +
        ((!!state_matrix[GAMEPAD_HOME])   << 14)
    );
    // Adjust range from [-1,1] to [-32767,32767].
    int16_t lx_report = hid_axis(gamepad_axis[LX], GAMEPAD_AXIS_LX, GAMEPAD_AXIS_LX_NEG) * BIT_15;
    int16_t ly_report = hid_axis(gamepad_axis[LY], GAMEPAD_AXIS_LY, GAMEPAD_AXIS_LY_NEG) * BIT_15;
    int16_t rx_report = hid_axis(gamepad_axis[RX], GAMEPAD_AXIS_RX, GAMEPAD_AXIS_RX_NEG) * BIT_15;
    int16_t ry_report = hid_axis(gamepad_axis[RY], GAMEPAD_AXIS_RY, GAMEPAD_AXIS_RY_NEG) * BIT_15;
    // HID triggers must be also defined as unsigned in the USB descriptor, and has to be manually
    // value-shifted from signed to unsigned here, otherwise Windows is having erratic behavior and
    // inconsistencies between games (not sure if a bug in Windows' DirectInput or TinyUSB).
    int16_t lz_report = ((hid_axis(gamepad_axis[LZ], GAMEPAD_AXIS_LZ, 0) * 2) - 1) * BIT_15;
    int16_t rz_report = ((hid_axis(gamepad_axis[RZ], GAMEPAD_AXIS_RZ, 0) * 2) - 1) * BIT_15;
    GamepadReport report = {
        lx_report,
        ly_report,
        rx_report,
        ry_report,
        lz_report,
        rz_report,
        buttons,
    };
    return report;
}

XInputReport hid_get_xinput_report() {
    int8_t buttons_0 = 0;
    int8_t buttons_1 = 0;
    // Button bitmask.
    // Any value bigger than 1 consolidates to 1 (with !!).
    for(int i=0; i<8; i++) {
        buttons_0 += (!!state_matrix[GAMEPAD_INDEX + i]) << i;
    }
    for(int i=0; i<8; i++) {
        buttons_1 += (!!state_matrix[GAMEPAD_INDEX + i + 8]) << i;
    }
    // Adjust range from [-1,1] to [-32767,32767].
    int16_t lx_report = hid_axis(gamepad_axis[LX], GAMEPAD_AXIS_LX, GAMEPAD_AXIS_LX_NEG) * BIT_15;
    int16_t ly_report = hid_axis(gamepad_axis[LY], GAMEPAD_AXIS_LY, GAMEPAD_AXIS_LY_NEG) * BIT_15;
    int16_t rx_report = hid_axis(gamepad_axis[RX], GAMEPAD_AXIS_RX, GAMEPAD_AXIS_RX_NEG) * BIT_15;
    int16_t ry_report = hid_axis(gamepad_axis[RY], GAMEPAD_AXIS_RY, GAMEPAD_AXIS_RY_NEG) * BIT_15;
    // Adjust range from [0,1] to [0,255].
    uint16_t lz_report = hid_axis(gamepad_axis[LZ], GAMEPAD_AXIS_LZ, 0) * BIT_8;
    uint16_t rz_report = hid_axis(gamepad_axis[RZ], GAMEPAD_AXIS_RZ, 0) * BIT_8;
    XInputReport report = {
        .report_id   = 0,
        .report_size = XINPUT_REPORT_SIZE,
        .buttons_0   = buttons_0,
        .buttons_1   = buttons_1,
        .lz          = lz_report,
        .rz          = rz_report,
        .lx          = lx_report,
        .ly          = -ly_report,
        .rx          = rx_report,
        .ry          = -ry_report,
        .reserved    = {0, 0, 0, 0, 0, 0}
    };
    return report;
}

void hid_reset_mouse() {
    mouse_x = 0;
    mouse_y = 0;
    state_matrix[MOUSE_SCROLL_UP] = 0;
    state_matrix[MOUSE_SCROLL_DOWN] = 0;
}

void hid_reset_gamepad_axis() {
    // Gamepad axis values being reset so potentially unsent values are not
    // aggregated with the next cycle.
    for(uint8_t i=0; i<6; i++) gamepad_axis[i] = 0;
}

void hid_set_gamepad_synced() {
    for(uint8_t i=0; i<6; i++) gamepad_axis_last[i] = gamepad_axis[i];
    synced_gamepad = true;
    priority_gamepad = 0;
}

void hid_evaluate_gamepad_synced() {
    // Only evaluate if wasn't already marked as unsynced (by buttons).
    if (!synced_gamepad) return;
    // Evaluate axis.
    for(uint8_t i=0; i<6; i++) {
        if (gamepad_axis[i] != gamepad_axis_last[i]) {
            synced_gamepad = false;
        }
    }
}

bool hid_report_keyboard(bool wired) {
    KeyboardReport report = hid_get_keyboard_report();
    if (wired) tud_hid_report(REPORT_KEYBOARD, &report, sizeof(report));
    else wireless_send_hid(REPORT_KEYBOARD, &report, sizeof(report));
    synced_keyboard = true;
}

bool hid_report_mouse(bool wired) {
    MouseReport report = hid_get_mouse_report();
    if (wired) tud_hid_report(REPORT_MOUSE, &report, sizeof(report));
    else wireless_send_hid(REPORT_MOUSE, &report, sizeof(report));
    hid_reset_mouse();
    synced_mouse = true;
    priority_mouse = 0;
}

bool hid_report_gamepad(bool wired) {
    GamepadReport report = hid_get_gamepad_report();
    if (wired) tud_hid_report(REPORT_GAMEPAD, &report, sizeof(report));
    else wireless_send_hid(REPORT_GAMEPAD, &report, sizeof(report));
    hid_set_gamepad_synced();
}

bool hid_report_xinput(bool wired) {
    XInputReport report = hid_get_xinput_report();
    if (wired) xinput_send_report(&report);
    else wireless_send_hid(REPORT_XINPUT, &report, sizeof(report));
    hid_set_gamepad_synced();
}

uint8_t hid_get_priority() {
    // Not all events are sent everytime, they are delivered based on their
    // priority ratio and how long they have been queueing.
    // For example thumbstick movement may be queued for some cycles if there
    // is a lot of mouse data being sent.
    hid_evaluate_gamepad_synced(); // Special case because accumulative absolute axis.
    if (!synced_mouse) priority_mouse += 1 * HID_REPORT_PRIORITY_RATIO;
    if (!synced_gamepad) priority_gamepad += 1;
    // Evaluate.
    if (!synced_keyboard) return REPORT_KEYBOARD;
    if (!synced_mouse && (priority_mouse > priority_gamepad)) return REPORT_MOUSE;
    if (!synced_gamepad) {
        if (config_get_protocol() == PROTOCOL_GENERIC) return REPORT_GAMEPAD;
        else return REPORT_XINPUT;
    }
    return 0;
}

bool hid_report_wired() {
    if (!hid_allow_communication) return true;
    uint8_t device_to_report = hid_get_priority();
    tud_task();
    if (tud_ready()) {
        if (tud_hid_ready()) {
            webusb_read();
            webusb_flush();
            if (device_to_report == REPORT_KEYBOARD) hid_report_keyboard(true);
            if (device_to_report == REPORT_MOUSE) hid_report_mouse(true);
            if (device_to_report == REPORT_GAMEPAD) hid_report_gamepad(true);
        }
        if (device_to_report == REPORT_XINPUT) {
            if (tud_suspended()) tud_remote_wakeup();
            hid_report_xinput(true);
        }
        hid_reset_gamepad_axis();
        return true;
    } else {
        return false;
    }
}

bool hid_report_wireless() {
    if (!hid_allow_communication) return true;
    uint8_t device_to_report = hid_get_priority();
    if (device_to_report == REPORT_KEYBOARD) hid_report_keyboard(false);
    if (device_to_report == REPORT_MOUSE) hid_report_mouse(false);
    if (device_to_report == REPORT_GAMEPAD) hid_report_gamepad(false);
    if (device_to_report == REPORT_XINPUT) hid_report_xinput(false);
    hid_reset_gamepad_axis();
    // webusb_read();
    webusb_flush();
    return true;
}

void hid_report_dongle(uint8_t report_id, uint8_t* payload) {
    tud_task();
    if (tud_ready()) {
        if (report_id == REPORT_KEYBOARD) {
            if (tud_hid_ready()) {
                tud_hid_report(REPORT_KEYBOARD, payload, sizeof(KeyboardReport));
            }
        }
        if (report_id == REPORT_MOUSE) {
            if (tud_hid_ready()) {
                tud_hid_report(REPORT_MOUSE, payload, sizeof(MouseReport));
            }
        }
        if (report_id == REPORT_GAMEPAD) {
            if (tud_hid_ready()) {
                tud_hid_report(REPORT_GAMEPAD, payload, sizeof(GamepadReport));
            }
        }
        if (report_id == REPORT_XINPUT) {
            xinput_send_report((XInputReport*)payload);
        }
    }
}

// A not-so-secret easter egg.
void hid_thanks_(alarm_id_t alarm) {
    cancel_alarm(alarm);
    static uint8_t x = 0;
    static bool p = 0;
    static uint8_t r;
    if (x == 0 && p == false) {
        r = random8() % thanks_len;
    }
    if (thanks_list[r][x] == 0) {
        x = 0;
        p = 0;
        return;
    }
    if (!p) {
        hid_press(thanks_list[r][x]);
        p = true;
    } else if (p) {
        hid_release(thanks_list[r][x]);
        p = false;
        x += 1;
    }
    add_alarm_in_ms(5, (alarm_callback_t)hid_thanks_, NULL, true);
}

void hid_thanks() {
    add_alarm_in_ms(5, (alarm_callback_t)hid_thanks_, NULL, true);
}

void hid_init() {
    info("INIT: HID\n");
    alarm_pool = alarm_pool_create(2, 255);
}
