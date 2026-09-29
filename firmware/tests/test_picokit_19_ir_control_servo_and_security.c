/**
 * FILE: test_picokit_19_ir_control_servo_and_security.c
 *
 * DESCRIPTION:
 * Native unit tests for the RP2350 Picokit servo position node:
 * provisioning constants, the packet artifact, CRC, the RYLR998 AT
 * interface, the VS1838B infrared decoder, the console angle parser, and
 * the servo position state machine that pairs each command with an
 * authenticated LoRa heartbeat.
 *
 * BRIEF:
 * Native unit test runner for picokit-19-ir-control-servo.
 *
 * AUTHOR: Kevin Thomas
 * DATE: September 2026
 */

#include "harness.h"
#include "mock/pico/stdlib.h"
#include "mock/pico/stdio.h"
#include "mock/pico/time.h"
#include "mock/hardware/gpio.h"
#include "mock/hardware/uart.h"
#include "picokit_19_ir_control_servo.h"
#include "crc.h"
#include "radio.h"
#include "ir_remote.h"
#include "monitor.h"
#include "status_led.h"
#include <string.h>

#include "../src/crc.c"
#include "../src/radio.c"
#include "../src/ir_remote.c"
#include "../src/monitor.c"

/**
 * @brief File-scope UART transmit capture buffer.
 */
static char s_tx[2048];

/**
 * @brief File-scope decoded inbound radio report.
 */
static radio_rcv_t s_rcv;

/**
 * @brief File-scope NEC pulse-duration scratch buffer.
 */
static uint16_t s_pulses[IR_REMOTE_MAX_PULSES];

/**
 * @brief File-scope GPIO timeline offset scratch buffer for the IR eye.
 */
static uint32_t s_ir_off[IR_REMOTE_MAX_PULSES * 2u];

/**
 * @brief File-scope GPIO timeline level scratch buffer for the IR eye.
 */
static int s_ir_lvl[IR_REMOTE_MAX_PULSES * 2u];

/**
 * @brief Reset every host mock peripheral.
 *
 * @param void No parameters.
 * @return void
 */
static void reset_all(void) {
    mock_timer_reset();
    mock_gpio_reset();
    mock_uart_reset();
    mock_stdio_reset();
}

/**
 * @brief Initialize the monitor and clear its provisioning UART traffic.
 *
 * @param void No parameters.
 * @return void
 */
static void init_monitor(void) {
    reset_all();
    TEST_ASSERT_TRUE(monitor_init());
    mock_uart_reset();
}

/**
 * @brief Service one monitor tick at an absolute fake-clock time.
 *
 * @param when_us Absolute fake-clock time in microseconds.
 * @return void
 */
static void step_at(uint64_t when_us) {
    mock_timer_set_us(when_us);
    monitor_step();
}

/**
 * @brief Build the NEC frame word for address zero and a command.
 *
 * @param command Eight-bit remote command code.
 * @return uint32_t LSB-first frame word with inverse bytes.
 */
static uint32_t nec_word(uint8_t command) {
    return 0x0000FF00u | ((uint32_t)command << 16u) |
           ((uint32_t)(uint8_t)~command << 24u);
}

/**
 * @brief Fill the thirty-two LSB-first mark and space durations.
 *
 * @param pulses Pointer to the pulse-duration buffer.
 * @param word LSB-first NEC frame word.
 * @return void
 */
static void nec_bits(uint16_t *pulses, uint32_t word) {
    uint8_t i;
    for (i = 0u; i < 32u; ++i) {
        pulses[2u + 2u * i] = 560u;
        pulses[3u + 2u * i] = ((word >> i) & 1u) ? 1690u : 560u;
    }
}

/**
 * @brief Fill a complete NEC pulse train for a command.
 *
 * @param pulses Pointer to the pulse-duration buffer.
 * @param command Eight-bit remote command code.
 * @return void
 */
static void nec_fill(uint16_t *pulses, uint8_t command) {
    pulses[0] = 9000u;
    pulses[1] = 4500u;
    nec_bits(pulses, nec_word(command));
    pulses[66] = 560u;
    pulses[67] = 560u;
}

/**
 * @brief Append one timeline point and advance the entry count.
 *
 * @param offsets Pointer to mutable offset array.
 * @param levels Pointer to mutable level array.
 * @param n Current entry count.
 * @param at Absolute offset in microseconds.
 * @param level Level to record.
 * @return size_t Updated entry count.
 */
static size_t nec_append(uint32_t *offsets, int *levels, size_t n,
                         uint32_t at, int level) {
    offsets[n] = at;
    levels[n] = level;
    return n + 1u;
}

/**
 * @brief Lay a NEC pulse train onto a mock GPIO timeline.
 *
 * @param pulses Pointer to the pulse-duration buffer.
 * @param offsets Pointer to mutable offset array.
 * @param levels Pointer to mutable level array.
 * @return size_t Number of timeline entries written.
 */
static size_t nec_place(const uint16_t *pulses, uint32_t *offsets,
                        int *levels) {
    size_t i;
    size_t n = 0u;
    uint32_t t = 0u;
    for (i = 0u; i < IR_REMOTE_MAX_PULSES; ++i) {
        n = nec_append(offsets, levels, n, t, (i % 2u == 0u) ? 0 : 1);
        t += pulses[i];
    }
    n = nec_append(offsets, levels, n, t, 0);
    return n;
}

/**
 * @brief Arm the mock GPIO timeline with a NEC frame.
 *
 * @param command Eight-bit remote command code.
 * @return void
 */
static void nec_arm(uint8_t command) {
    size_t count;
    nec_fill(s_pulses, command);
    count = nec_place(s_pulses, s_ir_off, s_ir_lvl);
    mock_gpio_timeline_begin_at(mock_timer_now_us(), s_ir_off, s_ir_lvl,
                                count, PICOKIT_19_IR_CONTROL_SERVO_IR_PIN);
}

/**
 * @brief Assert the GPIO pin and position provisioning constants.
 *
 * @param void No parameters.
 * @return void
 */
static void assert_pin_constants(void) {
    TEST_ASSERT_EQUAL_UINT(25u, PICOKIT_19_IR_CONTROL_SERVO_LED_PIN);
    TEST_ASSERT_EQUAL_UINT(8u, PICOKIT_19_IR_CONTROL_SERVO_UART_TX);
    TEST_ASSERT_EQUAL_UINT(9u, PICOKIT_19_IR_CONTROL_SERVO_UART_RX);
    TEST_ASSERT_EQUAL_UINT(16u, PICOKIT_19_IR_CONTROL_SERVO_RED_LED_PIN);
    TEST_ASSERT_EQUAL_UINT(18u, PICOKIT_19_IR_CONTROL_SERVO_YELLOW_LED_PIN);
    TEST_ASSERT_EQUAL_UINT(17u, PICOKIT_19_IR_CONTROL_SERVO_GREEN_LED_PIN);
    TEST_ASSERT_EQUAL_UINT(5u, PICOKIT_19_IR_CONTROL_SERVO_IR_PIN);
}

/**
 * @brief Assert the UART and frame provisioning constants.
 *
 * @param void No parameters.
 * @return void
 */
static void assert_frame_constants(void) {
    TEST_ASSERT_EQUAL_UINT(115200u, PICOKIT_19_IR_CONTROL_SERVO_UART_BAUD);
    TEST_ASSERT_EQUAL_UINT(48u, PICOKIT_19_IR_CONTROL_SERVO_FRAME_SIZE);
    TEST_ASSERT_EQUAL_UINT(5000u, PICOKIT_19_IR_CONTROL_SERVO_TX_INTERVAL_MS);
    TEST_ASSERT_EQUAL_UINT(3u, STATUS_LED_STEP_COUNT);
}

/**
 * @brief Assert the packet artifact identity constants.
 *
 * @param void No parameters.
 * @return void
 */
static void assert_artifact_ids(void) {
    TEST_ASSERT_EQUAL_UINT(1u, PACKET_FRAME_VERSION);
    TEST_ASSERT_EQUAL_UINT(19u, PACKET_NODE_ID);
    TEST_ASSERT_EQUAL_HEX16(0x0001u, PACKET_HUB_ADDRESS);
    TEST_ASSERT_EQUAL_UINT(48u, PACKET_FRAME_SIZE);
    TEST_ASSERT_EQUAL_UINT(5000u, PACKET_TX_INTERVAL_MS);
}

/**
 * @brief Assert the packet artifact example heartbeat frame.
 *
 * @param void No parameters.
 * @return void
 */
static void assert_artifact_example(void) {
    TEST_ASSERT_EQUAL_UINT(48u, (unsigned)sizeof(PACKET_EXAMPLE_FRAME));
    TEST_ASSERT_EQUAL_UINT8(0x7Bu, PACKET_EXAMPLE_FRAME[0]);
    TEST_ASSERT_EQUAL_UINT8(0x31u, PACKET_EXAMPLE_FRAME[5]);
    TEST_ASSERT_EQUAL_UINT8(0x30u, PACKET_EXAMPLE_FRAME[18]);
    TEST_ASSERT_EQUAL_UINT8(0x7Du, PACKET_EXAMPLE_FRAME[19]);
    TEST_ASSERT_EQUAL_UINT8(0x00u, PACKET_EXAMPLE_FRAME[20]);
}

/**
 * @brief Parse the canonical comma-laden JSON +RCV line.
 *
 * @param void No parameters.
 * @return radio_result_t Parsed result code.
 */
static radio_result_t parse_json_rcv(void) {
    return radio_parse_rcv("+RCV=0002,10,{\"cmd\":91},-78,5", &s_rcv);
}

/**
 * @brief Assert +RCV parsing of a comma-laden JSON payload.
 *
 * @param void No parameters.
 * @return void
 */
static void assert_rcv_json(void) {
    TEST_ASSERT_EQUAL(RADIO_RESULT_OK, parse_json_rcv());
    TEST_ASSERT_EQUAL_HEX16(0x0002u, s_rcv.sender);
    TEST_ASSERT_EQUAL_UINT(10u, (unsigned)s_rcv.len);
    TEST_ASSERT_EQUAL_STRING("{\"cmd\":91}", s_rcv.payload);
    TEST_ASSERT_EQUAL_INT(-78, s_rcv.rssi);
    TEST_ASSERT_EQUAL_INT(5, s_rcv.snr);
}

/**
 * @brief Assert +RCV parsing of a short comma-bearing payload.
 *
 * @param void No parameters.
 * @return void
 */
static void assert_rcv_comma(void) {
    TEST_ASSERT_EQUAL(RADIO_RESULT_OK, radio_parse_rcv("+RCV=0008,9,{\"a\",\"b\"},-60,3", &s_rcv));
    TEST_ASSERT_EQUAL_HEX16(0x0008u, s_rcv.sender);
    TEST_ASSERT_EQUAL_STRING("{\"a\",\"b\"}", s_rcv.payload);
}

/**
 * @brief Assert +RCV parsing of a frame with no RSSI/SNR tail.
 *
 * @param void No parameters.
 * @return void
 */
static void assert_rcv_no_tail(void) {
    TEST_ASSERT_EQUAL(RADIO_RESULT_OK, radio_parse_rcv("+RCV=0002,2,ok", &s_rcv));
    TEST_ASSERT_EQUAL_INT(0, s_rcv.rssi);
    TEST_ASSERT_EQUAL_INT(0, s_rcv.snr);
}

/**
 * @brief Assert +RCV rejection of malformed, oversized, and null lines.
 *
 * @param void No parameters.
 * @return void
 */
static void assert_rcv_rejects(void) {
    TEST_ASSERT_EQUAL(RADIO_RESULT_PARSE_ERROR, radio_parse_rcv("AT+SEND=0001,3,abc", &s_rcv));
    TEST_ASSERT_EQUAL(RADIO_RESULT_OVERSIZE, radio_parse_rcv("+RCV=0001,300,abcdef", &s_rcv));
    TEST_ASSERT_EQUAL(RADIO_RESULT_PARSE_ERROR, radio_parse_rcv("+RCV=0001,5,abc", &s_rcv));
    TEST_ASSERT_EQUAL(RADIO_RESULT_PARSE_ERROR, radio_parse_rcv(NULL, &s_rcv));
    TEST_ASSERT_EQUAL(RADIO_RESULT_PARSE_ERROR, radio_parse_rcv("+RCV=0001,1,a", NULL));
}

/**
 * @brief Assert the inbound line pump consumes two CRLF-terminated lines.
 *
 * @param line Pointer to line buffer.
 * @param len Pointer to accumulated length.
 * @return void
 */
static void assert_pump_lines(char *line, size_t *len) {
    TEST_ASSERT_TRUE(radio_line_pump(uart0, line, len));
    TEST_ASSERT_EQUAL_STRING("ab", line);
    TEST_ASSERT_TRUE(radio_line_pump(uart0, line, len));
    TEST_ASSERT_EQUAL_STRING("cd", line);
    TEST_ASSERT_FALSE(radio_line_pump(uart0, line, len));
}

/**
 * @brief Locate the hex payload after the AT+SEND address and length fields.
 *
 * @param void No parameters.
 * @return char* Pointer to the hex payload inside the captured frame.
 */
static char *tx_hex_start(void) {
    char *send = strstr(s_tx, "AT+SEND=");
    char *first = strchr(send, ',');
    char *second = strchr(first + 1, ',');
    return second + 1;
}

/**
 * @brief Trim the trailing CRLF from the captured hex payload.
 *
 * @param hex Pointer to the mutable hex payload.
 * @return size_t Number of remaining hex characters.
 */
static size_t tx_hex_len(char *hex) {
    size_t n = strlen(hex);
    while (n > 0u && (hex[n - 1u] == '\r' || hex[n - 1u] == '\n')) {
        n -= 1u;
    }
    hex[n] = '\0';
    return n;
}

/**
 * @brief Report whether every character is a lowercase hex digit.
 *
 * @param hex Pointer to the NUL-terminated candidate text.
 * @return bool true when the text is entirely lowercase hexadecimal.
 */
static bool tx_all_hex(const char *hex) {
    size_t i;
    for (i = 0u; hex[i] != '\0'; ++i) {
        if (hex_digit(hex[i]) < 0 || (hex[i] >= 'A' && hex[i] <= 'F')) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Assert the transmitted payload is an even-length lowercase hex envelope.
 *
 * @param void No parameters.
 * @return void
 */
static void assert_hex_payload(void) {
    char *hex = tx_hex_start();
    size_t hex_len = tx_hex_len(hex);
    TEST_ASSERT_TRUE(hex_len > 48u);
    TEST_ASSERT_TRUE((hex_len % 2u) == 0u);
    TEST_ASSERT_TRUE(tx_all_hex(hex));
}

/**
 * @brief Step one sweep tick and assert the resulting angle.
 *
 * @param when_us Absolute fake-clock time in microseconds.
 * @param expected_angle Expected angle after the tick.
 * @return void
 */
static void sweep_next(uint64_t when_us, unsigned expected_angle) {
    step_at(when_us);
    TEST_ASSERT_EQUAL_UINT(expected_angle, (unsigned)g_angle);
}

void test_config_constants(void) {
    assert_pin_constants();
    assert_frame_constants();
}

void test_packet_artifact_constants(void) {
    assert_artifact_ids();
    assert_artifact_example();
}

void test_crc16_ccitt(void) {
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, crc16_ccitt((const uint8_t *)"", 0u));
    TEST_ASSERT_EQUAL_HEX16(0x29B1u, crc16_ccitt((const uint8_t *)"123456789", 9u));
}

void test_radio_build_send_cmd(void) {
    char cmd[64];
    TEST_ASSERT_EQUAL(RADIO_RESULT_OK, radio_build_send_cmd(0x0001u, (const uint8_t *)"abc", 3u, cmd, sizeof(cmd)));
    TEST_ASSERT_EQUAL_STRING("AT+SEND=0001,3,abc\r\n", cmd);
    TEST_ASSERT_EQUAL(RADIO_RESULT_OVERSIZE, radio_build_send_cmd(0x0001u, (const uint8_t *)"abc", 257u, cmd, sizeof(cmd)));
    TEST_ASSERT_EQUAL(RADIO_RESULT_PARSE_ERROR, radio_build_send_cmd(0x0001u, NULL, 3u, cmd, sizeof(cmd)));
}

void test_radio_build_send_cmd_oversize_cmd(void) {
    char tiny[16];
    TEST_ASSERT_EQUAL(RADIO_RESULT_OVERSIZE, radio_build_send_cmd(0x0001u, (const uint8_t *)"abcdefghijklmnopqrst", 20u, tiny, sizeof(tiny)));
}

void test_radio_send_frame_oversize(void) {
    TEST_ASSERT_EQUAL(RADIO_RESULT_OVERSIZE, radio_send_frame(uart0, (const uint8_t *)"x", 257u));
}

void test_radio_parse_rcv(void) {
    assert_rcv_json();
    assert_rcv_comma();
    assert_rcv_no_tail();
}

void test_radio_parse_rcv_rejects(void) {
    assert_rcv_rejects();
}

void test_radio_line_pump(void) {
    char line[32];
    size_t len = 0u;
    mock_uart_reset();
    mock_uart_set_rx("ab\r\ncd\r\n", 8u);
    assert_pump_lines(line, &len);
}

void test_radio_hex_digits(void) {
    TEST_ASSERT_EQUAL(RADIO_RESULT_OK, radio_parse_rcv("+RCV=00a7,2,ok,-3,2", &s_rcv));
    TEST_ASSERT_EQUAL_HEX16(0x00A7u, s_rcv.sender);
    TEST_ASSERT_EQUAL(RADIO_RESULT_OK, radio_parse_rcv("+RCV=00FE,2,ok,-3,2", &s_rcv));
    TEST_ASSERT_EQUAL_HEX16(0x00FEu, s_rcv.sender);
}

void test_radio_parse_missing_commas(void) {
    TEST_ASSERT_EQUAL(RADIO_RESULT_PARSE_ERROR, radio_parse_rcv("+RCV=007ZX,3,hi,-1,1", &s_rcv));
    TEST_ASSERT_EQUAL(RADIO_RESULT_PARSE_ERROR, radio_parse_rcv("+RCV=0002,9Z,hi,-1,1", &s_rcv));
}

void test_radio_spoofed_sender_attribution(void) {
    TEST_ASSERT_EQUAL(RADIO_RESULT_OK, radio_parse_rcv("+RCV=0002,7,{\"a\",1},-90,3", &s_rcv));
    TEST_ASSERT_TRUE(radio_frame_is_from(&s_rcv, 0x0002u));
    TEST_ASSERT_FALSE(radio_frame_is_from(&s_rcv, 0x0008u));
}

void test_monitor_build_frame_json(void) {
    char frame[PICOKIT_19_IR_CONTROL_SERVO_FRAME_SIZE];
    g_seq = 12u;
    g_angle = 90u;
    monitor_build_frame(frame, sizeof(frame));
    TEST_ASSERT_EQUAL_STRING("{\"n\":19,\"s\":12,\"a\":90}", frame);
}

void test_monitor_init(void) {
    reset_all();
    TEST_ASSERT_TRUE(monitor_init());
    TEST_ASSERT_EQUAL_UINT(115200u, s_mock_uart_baud);
    TEST_ASSERT_TRUE(s_mock_gpio_dirs[PICOKIT_19_IR_CONTROL_SERVO_LED_PIN]);
    TEST_ASSERT_FALSE(s_mock_gpio_dirs[PICOKIT_19_IR_CONTROL_SERVO_IR_PIN]);
}

/**
 * @brief Feed one console line and assert the resulting commanded angle.
 *
 * @param input Console input string to supply.
 * @param len Number of characters in input.
 * @param at Absolute fake-clock time in microseconds.
 * @param expected Expected commanded angle after the tick.
 * @return void
 */
static void console_angle(const char *input, size_t len, uint64_t at,
                          unsigned expected) {
    mock_stdio_set_input(input, len);
    step_at(at);
    TEST_ASSERT_EQUAL_UINT(expected, (unsigned)g_angle);
}

void test_monitor_console(void) {
    uint64_t base;
    init_monitor();
    base = mock_timer_now_us();
    console_angle("90\n", 3u, base, 90u);
    console_angle("7x\n", 3u, base + 1000000u, 7u);
    console_angle("99999\n", 6u, base + 2000000u, SERVO_MAX_ANGLE_DEGREES);
}

/**
 * @brief Arm one NEC preset key and step the monitor at a given time.
 *
 * @param key Eight-bit remote command code.
 * @param when_us Absolute fake-clock time in microseconds.
 * @return void
 */
static void ir_step(uint8_t key, uint64_t when_us) {
    mock_timer_set_us(when_us);
    nec_arm(key);
    step_at(when_us);
}

void test_monitor_ir(void) {
    init_monitor();
    ir_step(0x47u, 1000000u);
    TEST_ASSERT_EQUAL_UINT(180u, (unsigned)g_angle);
}

void test_monitor_ir_presets(void) {
    init_monitor();
    ir_step(0x45u, 1000000u);
    ir_step(0x46u, 1200000u);
    TEST_ASSERT_EQUAL_UINT(90u, (unsigned)g_angle);
}

void test_monitor_ir_ignore(void) {
    init_monitor();
    ir_step(0x44u, 1000000u);
    TEST_ASSERT_EQUAL_UINT(0u, (unsigned)g_angle);
}

void test_monitor_transmit_frame(void) {
    size_t tx_len;
    init_monitor();
    step_at(mock_timer_now_us() + 5000000u);
    tx_len = mock_uart_get_tx(s_tx, sizeof(s_tx) - 1u);
    s_tx[tx_len] = '\0';
    TEST_ASSERT_TRUE(strstr(s_tx, "AT+SEND=0001,") != NULL);
    assert_hex_payload();
}

void test_monitor_transmit_no_key(void) {
    init_monitor();
    g_key_ready = false;
    mock_uart_reset();
    monitor_transmit();
    TEST_ASSERT_EQUAL_UINT(0u, (unsigned)mock_uart_get_tx(s_tx, sizeof(s_tx) - 1u));
    g_key_ready = true;
}

void test_monitor_not_ready(void) {
    monitor_deinit();
    TEST_ASSERT_FALSE(monitor_step());
}

void test_monitor_step_rx(void) {
    init_monitor();
    mock_uart_set_rx("+RCV=0002,10,{\"cmd\":91},-58,4\r\n", sizeof("+RCV=0002,10,{\"cmd\":91},-58,4\r\n") - 1u);
    step_at(mock_timer_now_us());
    TEST_ASSERT_EQUAL_UINT((unsigned)s_mock_rx_len, (unsigned)s_mock_rx_pos);
}

void test_ir_init(void) {
    reset_all();
    TEST_ASSERT_TRUE(ir_remote_init());
    TEST_ASSERT_FALSE(s_mock_gpio_dirs[PICOKIT_19_IR_CONTROL_SERVO_IR_PIN]);
}

void test_ir_decode_valid(void) {
    ir_command_t cmd;
    reset_all();
    nec_fill(s_pulses, 0x45u);
    TEST_ASSERT_TRUE(ir_decode_nec(s_pulses, IR_REMOTE_MAX_PULSES, &cmd));
    TEST_ASSERT_TRUE(cmd.valid);
    TEST_ASSERT_EQUAL_UINT8(0x00u, cmd.address);
    TEST_ASSERT_EQUAL_UINT8(0x45u, cmd.command);
}

void test_ir_decode_rejects(void) {
    ir_command_t cmd;
    reset_all();
    nec_fill(s_pulses, 0x45u);
    TEST_ASSERT_FALSE(ir_decode_nec(s_pulses, IR_REMOTE_MAX_PULSES - 1u, &cmd));
    TEST_ASSERT_FALSE(ir_decode_nec(NULL, IR_REMOTE_MAX_PULSES, &cmd));
    TEST_ASSERT_FALSE(ir_decode_nec(s_pulses, IR_REMOTE_MAX_PULSES, NULL));
}

void test_ir_decode_bad_leader(void) {
    ir_command_t cmd;
    reset_all();
    nec_fill(s_pulses, 0x45u);
    s_pulses[0] = 500u;
    TEST_ASSERT_FALSE(ir_decode_nec(s_pulses, IR_REMOTE_MAX_PULSES, &cmd));
}

void test_ir_decode_bad_mark(void) {
    ir_command_t cmd;
    reset_all();
    nec_fill(s_pulses, 0x45u);
    s_pulses[2] = 100u;
    TEST_ASSERT_FALSE(ir_decode_nec(s_pulses, IR_REMOTE_MAX_PULSES, &cmd));
}

void test_ir_decode_ambiguous(void) {
    ir_command_t cmd;
    reset_all();
    nec_fill(s_pulses, 0x45u);
    s_pulses[3] = 1100u;
    TEST_ASSERT_FALSE(ir_decode_nec(s_pulses, IR_REMOTE_MAX_PULSES, &cmd));
}

void test_ir_decode_bad_address(void) {
    ir_command_t cmd;
    reset_all();
    nec_fill(s_pulses, 0x45u);
    s_pulses[19] = 560u;
    TEST_ASSERT_FALSE(ir_decode_nec(s_pulses, IR_REMOTE_MAX_PULSES, &cmd));
}

void test_ir_decode_bad_command(void) {
    ir_command_t cmd;
    reset_all();
    nec_fill(s_pulses, 0x45u);
    s_pulses[51] = 1690u;
    TEST_ASSERT_FALSE(ir_decode_nec(s_pulses, IR_REMOTE_MAX_PULSES, &cmd));
}

void test_ir_poll_valid(void) {
    ir_command_t cmd;
    reset_all();
    nec_arm(0x45u);
    TEST_ASSERT_TRUE(ir_remote_poll(&cmd));
    TEST_ASSERT_EQUAL_UINT8(0x45u, cmd.command);
}

void test_ir_poll_timeout(void) {
    ir_command_t cmd;
    reset_all();
    TEST_ASSERT_FALSE(ir_remote_poll(&cmd));
}

void test_ir_poll_stuck_high(void) {
    ir_command_t cmd;
    reset_all();
    gpio_put(PICOKIT_19_IR_CONTROL_SERVO_IR_PIN, true);
    TEST_ASSERT_FALSE(ir_remote_poll(&cmd));
}

void setUp(void) {
    reset_all();
}

void tearDown(void) {
}

/**
 * @brief Run the provisioning, artifact, and CRC tests.
 *
 * @param void No parameters.
 * @return void
 */
static void run_basic_tests(void) {
    RUN_TEST(test_config_constants);
    RUN_TEST(test_packet_artifact_constants);
    RUN_TEST(test_crc16_ccitt);
}

/**
 * @brief Run the radio protocol tests.
 *
 * @param void No parameters.
 * @return void
 */
static void run_radio_tests(void) {
    RUN_TEST(test_radio_build_send_cmd);
    RUN_TEST(test_radio_build_send_cmd_oversize_cmd);
    RUN_TEST(test_radio_send_frame_oversize);
    RUN_TEST(test_radio_parse_rcv);
    RUN_TEST(test_radio_parse_rcv_rejects);
}

/**
 * @brief Run the inbound line and spoof-surface radio tests.
 *
 * @param void No parameters.
 * @return void
 */
static void run_radio_edge_tests(void) {
    RUN_TEST(test_radio_line_pump);
    RUN_TEST(test_radio_hex_digits);
    RUN_TEST(test_radio_parse_missing_commas);
    RUN_TEST(test_radio_spoofed_sender_attribution);
}

/**
 * @brief Run the servo position monitor state-machine tests.
 *
 * @param void No parameters.
 * @return void
 */
static void run_monitor_tests(void) {
    RUN_TEST(test_monitor_build_frame_json);
    RUN_TEST(test_monitor_init);
    RUN_TEST(test_monitor_console);
    RUN_TEST(test_monitor_ir);
    RUN_TEST(test_monitor_ir_presets);
    RUN_TEST(test_monitor_ir_ignore);
}

/**
 * @brief Run the remaining monitor transmit and receive tests.
 *
 * @param void No parameters.
 * @return void
 */
static void run_monitor_extra_tests(void) {
    RUN_TEST(test_monitor_transmit_frame);
    RUN_TEST(test_monitor_transmit_no_key);
    RUN_TEST(test_monitor_not_ready);
    RUN_TEST(test_monitor_step_rx);
}

/**
 * @brief Run every monitor state-machine test group.
 *
 * @param void No parameters.
 * @return void
 */
static void run_all_monitor_tests(void) {
    run_monitor_tests();
    run_monitor_extra_tests();
}

/**
 * @brief Run the infrared decoder tests.
 *
 * @param void No parameters.
 * @return void
 */
static void run_ir_decode_tests(void) {
    RUN_TEST(test_ir_init);
    RUN_TEST(test_ir_decode_valid);
    RUN_TEST(test_ir_decode_rejects);
    RUN_TEST(test_ir_decode_bad_leader);
    RUN_TEST(test_ir_decode_bad_mark);
    RUN_TEST(test_ir_decode_ambiguous);
    RUN_TEST(test_ir_decode_bad_address);
    RUN_TEST(test_ir_decode_bad_command);
}

/**
 * @brief Run the infrared polling tests.
 *
 * @param void No parameters.
 * @return void
 */
static void run_ir_poll_tests(void) {
    RUN_TEST(test_ir_poll_valid);
    RUN_TEST(test_ir_poll_timeout);
    RUN_TEST(test_ir_poll_stuck_high);
}

/**
 * @brief Run all infrared decoder and polling tests.
 *
 * @param void No parameters.
 * @return void
 */
static void run_ir_tests(void) {
    run_ir_decode_tests();
    run_ir_poll_tests();
}

/**
 * @brief Run the peripheral and security module test groups.
 *
 * @param void No parameters.
 * @return void
 */
extern void run_peripheral_and_crypto_tests(void);

int main(void) {
    TEST_BEGIN();
    run_basic_tests();
    run_radio_tests();
    run_radio_edge_tests();
    run_all_monitor_tests();
    run_ir_tests();
    run_peripheral_and_crypto_tests();
    return TEST_END();
}
