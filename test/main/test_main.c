/*
 * P1P2 Protocol Unit Tests
 *
 * Tests the F-series decode and control response logic
 * using Unity framework. Runs on ESP32-C6 target.
 *
 * Build: cd test && idf.py set-target esp32c6 && idf.py build
 * Flash: idf.py -p PORT flash monitor
 */

#include <string.h>
#include <stdio.h>
#include "unity.h"
#include "p1p2_protocol.h"
#include "p1p2_fseries.h"
#include "p1p2_bus_types.h"

/* External decode function from p1p2_fseries_decode.c */
extern void p1p2_fseries_decode_packet(const p1p2_packet_t *pkt, p1p2_hvac_state_t *state);

/* External control functions from p1p2_fseries_control.c */
extern void    p1p2_fseries_control_init(int model);
extern uint8_t p1p2_fseries_build_response_38(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_3b(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_39(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_3a(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_3c(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_empty(const uint8_t *rb, uint8_t rb_len,
                                                  uint8_t *wb, uint8_t wb_max);
extern esp_err_t p1p2_fseries_queue_write(uint8_t packet_type, uint8_t payload_offset,
                                           uint8_t value, uint8_t mask, uint8_t count);
extern esp_err_t p1p2_fseries_apply_command(const p1p2_control_cmd_t *cmd);

/* ================================================================
 * Helper: build a test packet
 * ================================================================ */
static p1p2_packet_t make_packet(const uint8_t *data, uint8_t len)
{
    p1p2_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.data, data, len);
    pkt.length = len;
    pkt.has_error = false;
    return pkt;
}

/* ================================================================
 * DECODE TESTS
 * ================================================================ */

TEST_CASE("decode: packet too short is ignored", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {0x00, 0x00, 0x10}; /* only 3 bytes, need at least 4 */
    p1p2_packet_t pkt = make_packet(raw, 3);

    p1p2_fseries_decode_packet(&pkt, &state);
    TEST_ASSERT_FALSE(state.data_valid);
}

TEST_CASE("decode: 0x10 status packet — power on, cool mode, 24C", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    /* src=0x00, dst=0x80, type=0x10, payload..., CRC */
    uint8_t raw[] = {
        0x00, 0x80, 0x10,       /* header */
        0x01,                    /* [0] power ON */
        0x00,                    /* [1] */
        F_MODE_COOL,             /* [2] mode = cool */
        0x00,                    /* [3] */
        24,                      /* [4] target cool temp = 24C */
        0x00,                    /* [5] */
        F_FAN_MED,               /* [6] fan cool = medium (0x31) */
        0x00,                    /* [7] */
        22,                      /* [8] target heat temp = 22C */
        0x00,                    /* [9] */
        F_FAN_LOW,               /* [10] fan heat = low (0x11) */
        0xAA,                    /* CRC (dummy) */
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_TRUE(state.data_valid);
    TEST_ASSERT_TRUE(state.power);
    TEST_ASSERT_EQUAL(P1P2_MODE_COOL, state.mode);
    TEST_ASSERT_EQUAL(240, state.target_temp_cool);  /* 24 * 10 */
    TEST_ASSERT_EQUAL(220, state.target_temp_heat);  /* 22 * 10 */
    TEST_ASSERT_EQUAL(P1P2_FAN_MED, state.fan_mode_cool);
    TEST_ASSERT_EQUAL(P1P2_FAN_LOW, state.fan_mode_heat);
    TEST_ASSERT_EQUAL(P1P2_RUNNING_COOLING, state.running);
}

TEST_CASE("decode: 0x10 status — power off → idle", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x00,                    /* power OFF */
        0x00, F_MODE_HEAT, 0x00,
        24, 0x00, F_FAN_LOW, 0x00,
        22, 0x00, F_FAN_LOW,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_FALSE(state.power);
    TEST_ASSERT_EQUAL(P1P2_RUNNING_IDLE, state.running);
}

TEST_CASE("decode: 0x10 heat mode → running heating", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01,                    /* power ON */
        0x00, F_MODE_HEAT, 0x00,
        24, 0x00, F_FAN_LOW, 0x00,
        22, 0x00, F_FAN_LOW,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_TRUE(state.power);
    TEST_ASSERT_EQUAL(P1P2_MODE_HEAT, state.mode);
    TEST_ASSERT_EQUAL(P1P2_RUNNING_HEATING, state.running);
}

TEST_CASE("decode: 0x11 temperature packet", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x11,
        23,                      /* [0] room temp = 23C */
        0x00,                    /* [1] */
        (uint8_t)(-5),           /* [2] outdoor temp = -5C (signed) */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(230, state.room_temp);     /* 23 * 10 */
    TEST_ASSERT_EQUAL(-50, state.outdoor_temp);  /* -5 * 10 */
}

TEST_CASE("decode: 0x14 compressor frequency", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x14,
        0x00, 0x3C,             /* compressor freq = 60 Hz */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(60, state.compressor_freq);
}

TEST_CASE("decode: 0xA3 counter packet", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0xA3,
        0x00, 0x00, 0x10, 0x00, /* operation hours = 4096 */
        0x00, 0x00, 0x00, 0x64, /* compressor starts = 100 */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(4096, state.operation_hours);
    TEST_ASSERT_EQUAL(100, state.compressor_starts);
}

TEST_CASE("decode: fan speed encoding", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    /* Test high fan speed (0x51 = bits 6:5 = 10) */
    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01, 0x00, F_MODE_COOL, 0x00,
        24, 0x00, F_FAN_HIGH, 0x00,
        22, 0x00, F_FAN_HIGH,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(P1P2_FAN_HIGH, state.fan_mode_cool);
    TEST_ASSERT_EQUAL(P1P2_FAN_HIGH, state.fan_mode_heat);
}

TEST_CASE("decode: all mode values", "[decode]")
{
    p1p2_hvac_state_t state;
    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01, 0x00, 0x00, 0x00,
        24, 0x00, F_FAN_LOW, 0x00,
        22, 0x00, F_FAN_LOW,
        0xAA,
    };

    /* Test each mode */
    struct { uint8_t raw; p1p2_system_mode_t expected; } modes[] = {
        {F_MODE_FAN,  P1P2_MODE_FAN},
        {F_MODE_HEAT, P1P2_MODE_HEAT},
        {F_MODE_COOL, P1P2_MODE_COOL},
        {F_MODE_AUTO, P1P2_MODE_AUTO},
        {F_MODE_DRY,  P1P2_MODE_DRY},
    };

    for (int i = 0; i < sizeof(modes)/sizeof(modes[0]); i++) {
        memset(&state, 0, sizeof(state));
        raw[5] = modes[i].raw;
        p1p2_packet_t pkt = make_packet(raw, sizeof(raw));
        p1p2_fseries_decode_packet(&pkt, &state);
        TEST_ASSERT_EQUAL_MESSAGE(modes[i].expected, state.mode,
                                  "Mode mismatch");
    }
}

TEST_CASE("decode: packet count increments", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {0x00, 0x80, 0x10, 0x01, 0xAA};
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);
    TEST_ASSERT_EQUAL(1, state.packet_count);

    p1p2_fseries_decode_packet(&pkt, &state);
    TEST_ASSERT_EQUAL(2, state.packet_count);
}

/* ================================================================
 * CONTROL RESPONSE TESTS
 * ================================================================ */

TEST_CASE("control: BCL 0x38 response — echo back state", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    /* Simulated 0x38 request from indoor unit (18+ bytes) */
    uint8_t rb[24] = {0};
    rb[0]  = 0x00;  /* src = main controller */
    rb[1]  = 0x40;  /* dst = aux controller */
    rb[2]  = 0x38;  /* type */
    rb[3]  = 0x01;  /* power ON */
    rb[5]  = F_MODE_COOL | F_MODE_ACTIVE_MASK;  /* mode = cool */
    rb[7]  = 24;    /* target cool temp */
    rb[9]  = F_FAN_MED; /* fan cool = medium */
    rb[11] = 22;    /* target heat temp */
    rb[13] = F_FAN_LOW; /* fan heat = low */
    rb[14] = 0x00;
    rb[18] = 0x03;  /* fan mode */

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(18, len);
    TEST_ASSERT_EQUAL(P1P2_ADDR_AUX_CTRL, wb[0]);  /* src = aux */
    TEST_ASSERT_EQUAL(0x00, wb[1]);                  /* dst = sender */
    TEST_ASSERT_EQUAL(0x38, wb[2]);                  /* same type */
    TEST_ASSERT_EQUAL(0x01, wb[3]);                  /* power echoed */
    TEST_ASSERT_EQUAL(24, wb[5]);                    /* temp echoed */
    TEST_ASSERT_EQUAL(22, wb[9]);                    /* heat temp echoed */
}

TEST_CASE("control: P model 0x38 response is 20 bytes", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_P);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; rb[5] = F_MODE_COOL;
    rb[7] = 24; rb[9] = F_FAN_LOW;
    rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(20, len);
    TEST_ASSERT_EQUAL(P1P2_ADDR_AUX_CTRL, wb[0]);
}

TEST_CASE("control: M model uses 0x3B, 22-byte response", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_M);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x3B;
    rb[3] = 0x01; rb[5] = F_MODE_HEAT;
    rb[7] = 24; rb[9] = F_FAN_LOW;
    rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[20] = 0x07; /* zones */
    rb[21] = 0x01; /* fan mode */

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_3b(rb, 22, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(22, len);
    TEST_ASSERT_EQUAL(0x07, wb[19]); /* zones echoed */
    TEST_ASSERT_EQUAL(0x01, wb[20]); /* fan mode echoed */
}

TEST_CASE("control: M model rejects 0x38", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_M);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01;

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(0, len); /* model M doesn't use 0x38 */
}

TEST_CASE("control: BCL model rejects 0x3B", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x3B;
    rb[3] = 0x01;

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_3b(rb, 22, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(0, len);
}

TEST_CASE("control: empty response (0x35/0x36/0x37)", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    uint8_t rb[] = {0x00, 0x40, 0x35, 0xAA};
    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_empty(rb, sizeof(rb), wb, sizeof(wb));

    TEST_ASSERT_EQUAL(3, len);
    TEST_ASSERT_EQUAL(P1P2_ADDR_AUX_CTRL, wb[0]);
    TEST_ASSERT_EQUAL(0x35, wb[2]);
}

/* ================================================================
 * PENDING WRITE TESTS
 * ================================================================ */

TEST_CASE("control: pending write applies to response", "[control][write]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    /* Queue a temperature change: cool temp = 26C */
    p1p2_control_cmd_t cmd = {
        .type = P1P2_CMD_SET_TEMP_COOL,
        .value = 260, /* 26.0C × 10 */
    };
    esp_err_t ret = p1p2_fseries_apply_command(&cmd);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Build response */
    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; rb[5] = F_MODE_COOL;
    rb[7] = 24; /* original temp 24C */
    rb[9] = F_FAN_LOW; rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(18, len);
    TEST_ASSERT_EQUAL(26, wb[5]); /* temperature overridden to 26C */
}

TEST_CASE("control: apply power command", "[control][write]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    p1p2_control_cmd_t cmd = {
        .type = P1P2_CMD_SET_POWER,
        .value = 0, /* power OFF */
    };
    esp_err_t ret = p1p2_fseries_apply_command(&cmd);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; /* currently ON */
    rb[5] = F_MODE_COOL; rb[7] = 24;
    rb[9] = F_FAN_LOW; rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};
    p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(0x00, wb[3]); /* power overridden to OFF */
}

/* ================================================================
 * CRC TEST
 * ================================================================ */

/* CRC calculation — duplicated here for testing (same as p1p2_bus.c) */
static uint8_t test_calc_crc(const uint8_t *data, uint8_t length,
                              uint8_t crc_gen, uint8_t crc_feed)
{
    uint8_t crc = crc_feed;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t c = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = ((crc ^ c) & 0x01) ? ((crc >> 1) ^ crc_gen) : (crc >> 1);
            c >>= 1;
        }
    }
    return crc;
}

TEST_CASE("CRC: Daikin F-series polynomial 0xD9", "[crc]")
{
    /* Known test vector: a simple status packet header */
    uint8_t data[] = {0x00, 0x00, 0x10};
    uint8_t crc = test_calc_crc(data, 3, F_SERIES_CRC_GEN, F_SERIES_CRC_FEED);

    /* CRC should be non-zero and deterministic */
    TEST_ASSERT_NOT_EQUAL(0, crc);

    /* Same input should always produce same CRC */
    uint8_t crc2 = test_calc_crc(data, 3, F_SERIES_CRC_GEN, F_SERIES_CRC_FEED);
    TEST_ASSERT_EQUAL(crc, crc2);
}

TEST_CASE("CRC: full packet CRC should verify to 0", "[crc]")
{
    /* Build a packet with valid CRC appended */
    uint8_t data[] = {0x00, 0x00, 0x10, 0x01, 0x00};
    uint8_t crc = test_calc_crc(data, 4, F_SERIES_CRC_GEN, F_SERIES_CRC_FEED);
    data[4] = crc;

    /* CRC over entire packet (including CRC byte) should be 0 for Daikin */
    uint8_t verify = test_calc_crc(data, 5, F_SERIES_CRC_GEN, F_SERIES_CRC_FEED);
    TEST_ASSERT_EQUAL(0, verify);
}

/* ================================================================
 * MAIN
 * ================================================================ */

void app_main(void)
{
    printf("\n\n");
    printf("========================================\n");
    printf("  P1P2MQTT Protocol Unit Tests\n");
    printf("========================================\n\n");

    UNITY_BEGIN();

    /* Decode tests */
    unity_run_test_by_name("decode: packet too short is ignored");
    unity_run_test_by_name("decode: 0x10 status packet — power on, cool mode, 24C");
    unity_run_test_by_name("decode: 0x10 status — power off → idle");
    unity_run_test_by_name("decode: 0x10 heat mode → running heating");
    unity_run_test_by_name("decode: 0x11 temperature packet");
    unity_run_test_by_name("decode: 0x14 compressor frequency");
    unity_run_test_by_name("decode: 0xA3 counter packet");
    unity_run_test_by_name("decode: fan speed encoding");
    unity_run_test_by_name("decode: all mode values");
    unity_run_test_by_name("decode: packet count increments");

    /* Control response tests */
    unity_run_test_by_name("control: BCL 0x38 response — echo back state");
    unity_run_test_by_name("control: P model 0x38 response is 20 bytes");
    unity_run_test_by_name("control: M model uses 0x3B, 22-byte response");
    unity_run_test_by_name("control: M model rejects 0x38");
    unity_run_test_by_name("control: BCL model rejects 0x3B");
    unity_run_test_by_name("control: empty response (0x35/0x36/0x37)");

    /* Pending write tests */
    unity_run_test_by_name("control: pending write applies to response");
    unity_run_test_by_name("control: apply power command");

    /* CRC tests */
    unity_run_test_by_name("CRC: Daikin F-series polynomial 0xD9");
    unity_run_test_by_name("CRC: full packet CRC should verify to 0");

    UNITY_END();

    printf("\n========================================\n");
    printf("  Tests complete\n");
    printf("========================================\n");
}
