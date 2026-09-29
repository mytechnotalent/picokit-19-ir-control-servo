// MIT License
//
// Copyright (c) 2026 Kevin Thomas
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Author:  Kevin Thomas
// Email:   kevin@mytechnotalent.com
// GitHub:  https://github.com/mytechnotalent/picokit-19-ir-control-servo
// File:    monitor.c
// Desc:    Implements the servo position state machine that moves the horn
//          to a position commanded from the infrared eye or the console.
// Created: 2026

#include "picokit_19_ir_control_servo.h"
#include "monitor.h"
#include "servo.h"
#include "ir_remote.h"
#include "radio.h"
#include "status_led.h"
#include "ccm.h"
#include "envelope.h"
#include "field_secrets.h"
#include "hardware/gpio.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief Module-ready flag.
 *
 * Set to true by monitor_init() once the peripherals are configured.
 * monitor_step() returns false while this flag is clear.
 */
static bool g_ready;

/**
 * @brief Current commanded servo angle in degrees.
 */
static uint8_t g_angle;

/**
 * @brief Monotonic transmit sequence number.
 */
static uint16_t g_seq;

/**
 * @brief Absolute time in microseconds of the next position report.
 */
static uint64_t g_next_step_us;

/**
 * @brief Absolute time in microseconds of the next authenticated transmit.
 */
static uint64_t g_next_tx_us;

/**
 * @brief Inbound radio line accumulator.
 */
static char g_rx_line[RADIO_LINE_BUF_LEN];

/**
 * @brief Number of bytes currently held in the inbound line accumulator.
 */
static size_t g_rx_len;

/**
 * @brief AES-128 session key for telemetry.
 */
static uint8_t g_key[CCM_KEY_LEN];

/**
 * @brief True once the telemetry session key has been loaded.
 */
static bool g_key_ready;

/**
 * @brief Console angle value under construction.
 */
static uint16_t g_console_value;

/**
 * @brief Number of decimal digits collected for the console angle.
 */
static uint8_t g_console_digits;

/**
 * @brief Configure the onboard heartbeat LED as a dark output.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_state_init_io(void) {
    gpio_init(PICOKIT_19_IR_CONTROL_SERVO_LED_PIN);
    gpio_set_dir(PICOKIT_19_IR_CONTROL_SERVO_LED_PIN, GPIO_OUT);
    gpio_put(PICOKIT_19_IR_CONTROL_SERVO_LED_PIN, 0);
}

/**
 * @brief Reset the angle, sequence, console, and transmit timing.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_state_init(void) {
    uint64_t now_us = time_us_64();
    g_angle = 0u;
    g_seq = 0u;
    g_console_value = 0u;
    g_console_digits = 0u;
    g_next_step_us = now_us;
    g_next_tx_us = now_us + (uint64_t)PICOKIT_19_IR_CONTROL_SERVO_TX_INTERVAL_MS * 1000u;
    g_ready = true;
}

/**
 * @brief Load the telemetry session key from the field secret.
 *
 * LAB-ONLY: production must provision the session key through OTP rather
 * than embedding a committed key.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_load_key(void) {
    static const uint8_t key[CCM_KEY_LEN] = FIELD_SECRET_KEY;
    memcpy(g_key, key, CCM_KEY_LEN);
    g_key_ready = true;
}

/**
 * @brief Print the boot banner for the position lesson.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_banner(void) {
    printf("=== PICOKIT-19 IR CONTROL SERVO // PRESET POSITION + HEARTBEAT ===\n");
}

/**
 * @brief Derive the field key and announce a ready monitor.
 *
 * @param void No parameters.
 * @return bool true when the field key was derived and installed.
 */
static bool monitor_finish(void) {
    monitor_load_key();
    monitor_banner();
    return true;
}

/**
 * @brief Blink the onboard heartbeat LED exactly once.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_heartbeat(void) {
    gpio_put(PICOKIT_19_IR_CONTROL_SERVO_LED_PIN, 1);
    sleep_us(MONITOR_HEARTBEAT_BLINK_US);
    gpio_put(PICOKIT_19_IR_CONTROL_SERVO_LED_PIN, 0);
    sleep_us(MONITOR_HEARTBEAT_BLINK_US);
}

/**
 * @brief Clamp and apply a commanded servo angle.
 *
 * @param requested Requested angle in degrees.
 * @return void
 */
static void monitor_set_angle(uint8_t requested) {
    if (requested > SERVO_MAX_ANGLE_DEGREES) {
        requested = SERVO_MAX_ANGLE_DEGREES;
    }
    g_angle = requested;
    servo_set_angle(g_angle);
}

/**
 * @brief Push one decimal digit into the console angle accumulator.
 *
 * @param c Character received from the console.
 * @return void
 */
static void monitor_console_push(int c) {
    if (c < '0' || c > '9') {
        return;
    }
    if (g_console_digits >= MONITOR_CONSOLE_MAX_DIGITS) {
        return;
    }
    g_console_value = (uint16_t)(g_console_value * 10u + (uint16_t)(c - '0'));
    g_console_digits += 1u;
}

/**
 * @brief Apply the accumulated console angle and clear the accumulator.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_console_submit(void) {
    monitor_set_angle((uint8_t)g_console_value);
    printf("SET angle=%u\n", (unsigned)g_angle);
    g_console_value = 0u;
    g_console_digits = 0u;
}

/**
 * @brief Process one console character.
 *
 * @param c Character received from the console.
 * @return void
 */
static void monitor_console_char(int c) {
    if (c == '\n' || c == '\r') {
        monitor_console_submit();
        return;
    }
    monitor_console_push(c);
}

/**
 * @brief Drain the console and process every pending character.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_console_tick(void) {
    int c = getchar_timeout_us(0u);
    while (c != PICO_ERROR_TIMEOUT) {
        monitor_console_char(c);
        c = getchar_timeout_us(0u);
    }
}

/**
 * @brief NEC command codes for the three preset servo angles.
 */
static const uint8_t g_preset_keys[3] = {
    MONITOR_KEY_POS_0, MONITOR_KEY_POS_90, MONITOR_KEY_POS_180,
};

/**
 * @brief Servo angles in degrees for the three preset keys.
 */
static const uint8_t g_preset_angles[3] = {
    0u, 90u, 180u,
};

/**
 * @brief Resolve a remote key code to a preset servo angle.
 *
 * @param cmd Decoded eight-bit remote command code.
 * @param angle Pointer to mutable resolved angle in degrees.
 * @return bool true when the key code maps to a preset angle.
 */
static bool monitor_preset_for(uint8_t cmd, uint8_t *angle) {
    uint8_t i;
    for (i = 0u; i < 3u; ++i) {
        if (g_preset_keys[i] == cmd) {
            *angle = g_preset_angles[i];
            return true;
        }
    }
    return false;
}

/**
 * @brief Apply one decoded infrared remote command to the servo.
 *
 * @param cmd Pointer to the decoded infrared command.
 * @return void
 */
static void monitor_ir_apply(const ir_command_t *cmd) {
    uint8_t angle;
    printf("IR 0x%02X\n", (unsigned)cmd->command);
    if (monitor_preset_for(cmd->command, &angle)) {
        monitor_set_angle(angle);
    }
}

/**
 * @brief Poll the infrared eye for a position command.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_ir_tick(void) {
    ir_command_t cmd;
    if (ir_remote_poll(&cmd)) {
        monitor_ir_apply(&cmd);
    }
}

/**
 * @brief Print one console line for the current commanded angle.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_log_step(void) {
    printf("POS angle=%u seq=%u\n", (unsigned)g_angle, (unsigned)g_seq);
}

/**
 * @brief Re-apply the commanded position and schedule the next report.
 *
 * @param now_us Current monotonic time in microseconds.
 * @return void
 */
static void monitor_step_tick(uint64_t now_us) {
    servo_set_angle(g_angle);
    monitor_heartbeat();
    monitor_log_step();
    g_next_step_us = now_us + (uint64_t)MONITOR_STEP_INTERVAL_MS * 1000u;
}

/**
 * @brief Format the heartbeat JSON body for the current angle.
 *
 * @param frame Pointer to the mutable frame output buffer.
 * @param frame_len Capacity of the frame output buffer in bytes.
 * @return size_t Number of JSON bytes written, or zero on overflow.
 */
static size_t monitor_build_frame(char *frame, size_t frame_len) {
    int written = snprintf(frame, frame_len, "{\"n\":%u,\"s\":%u,\"a\":%u}", (unsigned)PACKET_NODE_ID, (unsigned)g_seq, (unsigned)g_angle);
    return (written > 0 && (size_t)written < frame_len) ? (size_t)written : 0u;
}

/**
 * @brief Seal the current heartbeat body into a hex envelope.
 *
 * @param hex Pointer to the NUL-terminated hex output buffer.
 * @param hex_len Capacity of the hex output buffer in bytes.
 * @return bool true when the heartbeat was sealed and encoded.
 */
static bool monitor_seal_frame(char *hex, size_t hex_len) {
    char frame[PICOKIT_19_IR_CONTROL_SERVO_FRAME_SIZE];
    uint8_t nonce[ENVELOPE_NONCE_LEN];
    uint8_t ad = (uint8_t)PACKET_NODE_ID;
    size_t frame_len = monitor_build_frame(frame, sizeof(frame));
    envelope_fill_nonce(nonce);
    return envelope_seal_hex(g_key, nonce, &ad, 1u, (const uint8_t *)frame, frame_len, hex, hex_len);
}

/**
 * @brief Build and transmit the authenticated heartbeat frame.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_transmit(void) {
    char hex[ENVELOPE_MAX_HEX_LEN];
    if (!g_key_ready) {
        return;
    }
    if (monitor_seal_frame(hex, sizeof(hex))) {
        radio_send_frame(PICOKIT_19_IR_CONTROL_SERVO_UART, (const uint8_t *)hex, strlen(hex));
        g_seq += 1u;
    }
}

/**
 * @brief Transmit one heartbeat and schedule the next transmit.
 *
 * @param now_us Current monotonic time in microseconds.
 * @return void
 */
static void monitor_tx_tick(uint64_t now_us) {
    monitor_transmit();
    g_next_tx_us = now_us + (uint64_t)PICOKIT_19_IR_CONTROL_SERVO_TX_INTERVAL_MS * 1000u;
}

/**
 * @brief Drain inbound radio lines and log every valid +RCV report.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_rx_tick(void) {
    radio_rcv_t rcv;
    while (radio_line_pump(PICOKIT_19_IR_CONTROL_SERVO_UART, g_rx_line, &g_rx_len)) {
        if (radio_parse_rcv(g_rx_line, &rcv) == RADIO_RESULT_OK) {
            printf("RX from 0x%04X, %u bytes\n", (unsigned)rcv.sender, (unsigned)rcv.len);
        }
    }
}

/**
 * @brief Service the console, the infrared eye, and the radio receiver.
 *
 * @param void No parameters.
 * @return void
 */
static void monitor_service_inputs(void) {
    monitor_console_tick();
    monitor_ir_tick();
    monitor_rx_tick();
}

/**
 * @brief Service the position report and heartbeat transmit timers.
 *
 * @param now_us Current monotonic time in microseconds.
 * @return void
 */
static void monitor_service_timers(uint64_t now_us) {
    if (now_us >= g_next_step_us) {
        monitor_step_tick(now_us);
    }
    if (now_us >= g_next_tx_us) {
        monitor_tx_tick(now_us);
    }
}

bool monitor_init(void) {
    bool ok;
    ok = status_led_init() && radio_init(PICOKIT_19_IR_CONTROL_SERVO_UART);
    ok = ok && ir_remote_init();
    monitor_state_init_io();
    monitor_state_init();
    return ok && servo_init() && monitor_finish();
}

void monitor_deinit(void) {
    g_ready = false;
}

bool monitor_step(void) {
    uint64_t now_us;
    if (!g_ready) {
        return false;
    }
    now_us = time_us_64();
    monitor_service_inputs();
    monitor_service_timers(now_us);
    return true;
}
