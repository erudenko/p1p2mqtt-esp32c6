/*
 * P1P2 F-Series Decode — Extract HVAC data from P1/P2 bus packets
 *
 * Port of bytesbits2keyvalue() / bytes2keyvalue() from P1P2_ParameterConversion.h
 * for F-series VRV systems.
 *
 * Instead of converting to MQTT topic/value strings, this decoder updates
 * a p1p2_hvac_state_t structure that is read by the Matter layer.
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "p1p2_protocol.h"
#include "p1p2_fseries.h"
#include "p1p2_param_tables.h"

static const char *TAG = "p1p2_decode";

/*
 * Decode F-series fan speed byte to p1p2_fan_mode_t.
 * Fan encoding: bits 6-5 select speed.
 *   0x11 (00) = Low
 *   0x31 (01) = Medium
 *   0x51 (10) = High
 */
static p1p2_fan_mode_t decode_fan_speed(uint8_t raw)
{
    uint8_t speed_bits = (raw >> 5) & 0x03;
    switch (speed_bits) {
    case 0: return P1P2_FAN_LOW;
    case 1: return P1P2_FAN_MED;
    case 2: return P1P2_FAN_HIGH;
    default: return P1P2_FAN_AUTO;
    }
}

/*
 * Decode F-series operating mode byte to p1p2_system_mode_t.
 * Mode: low 3 bits.
 */
static p1p2_system_mode_t decode_mode(uint8_t raw)
{
    uint8_t mode_bits = raw & 0x07;
    switch (mode_bits) {
    case F_MODE_HEAT: return P1P2_MODE_HEAT;
    case F_MODE_COOL: return P1P2_MODE_COOL;
    case F_MODE_AUTO: return P1P2_MODE_AUTO;
    case F_MODE_FAN:  return P1P2_MODE_FAN;
    case F_MODE_DRY:  return P1P2_MODE_DRY;
    default:          return P1P2_MODE_OFF;
    }
}

/*
 * Decode a single F-series packet and update HVAC state.
 *
 * Packet layout:
 *   data[0] = source address
 *   data[1] = destination address
 *   data[2] = packet type
 *   data[3..length-2] = payload
 *   data[length-1] = CRC
 */
void p1p2_fseries_decode_packet(const p1p2_packet_t *pkt, p1p2_hvac_state_t *state)
{
    if (pkt->length < 4) return; /* minimum: src + dst + type + CRC */

    uint8_t src  = pkt->data[0];
    uint8_t dst  = pkt->data[1];
    uint8_t type = pkt->data[2];
    const uint8_t *payload = &pkt->data[3];
    uint8_t payload_len = pkt->length - 4; /* exclude src, dst, type, CRC */

    (void)src;
    (void)dst;

    switch (type) {
    case PKT_TYPE_STATUS_10:
        /*
         * Status packet 0x10: Power, mode, target temperatures, fan speeds.
         * This is the primary status packet from the indoor unit.
         *
         * Payload layout (varies by model, typical for BCL/M):
         *   [0] status flags (bit 0 = power on)
         *   [2] operating mode
         *   [4] target cooling temperature
         *   [6] fan speed cooling
         *   [8] target heating temperature
         *   [10] fan speed heating
         */
        if (payload_len >= 1)  state->power = (payload[0] & 0x01);
        if (payload_len >= 3)  state->mode  = decode_mode(payload[2]);
        if (payload_len >= 5)  state->target_temp_cool = payload[4] * 10;
        if (payload_len >= 7)  state->fan_mode_cool = decode_fan_speed(payload[6]);
        if (payload_len >= 9)  state->target_temp_heat = payload[8] * 10;
        if (payload_len >= 11) state->fan_mode_heat = decode_fan_speed(payload[10]);

        /* Determine running state from mode and power */
        if (!state->power) {
            state->running = P1P2_RUNNING_IDLE;
        } else if (state->mode == P1P2_MODE_HEAT) {
            state->running = P1P2_RUNNING_HEATING;
        } else if (state->mode == P1P2_MODE_COOL) {
            state->running = P1P2_RUNNING_COOLING;
        }

        state->data_valid = true;
        break;

    case PKT_TYPE_STATUS_11:
        /*
         * Temperature readings packet 0x11.
         *   [0] room/return temperature
         *   [2] outdoor temperature (signed)
         */
        if (payload_len >= 1) state->room_temp = payload[0] * 10;
        if (payload_len >= 3) state->outdoor_temp = (int8_t)payload[2] * 10;
        break;

    case PKT_TYPE_DATETIME_12:
        /* Date/time packet — we don't need this for Matter but could use for diagnostics */
        break;

    case PKT_TYPE_STATUS_14:
        /*
         * Extended status / compressor data.
         * Layout is model-dependent; common fields:
         *   [0-1] compressor frequency (16-bit)
         */
        if (payload_len >= 2) {
            state->compressor_freq = (payload[0] << 8) | payload[1];
        }
        break;

    case PKT_TYPE_CTRL_38:
        /*
         * 0x38 control exchange (request from indoor unit).
         * This packet is what we must respond to as auxiliary controller.
         * We also decode it to update our state with the "current request" values.
         *
         * Payload layout (from indoor unit, before our response):
         *   [0] status (bit 0 = power)
         *   [2] mode
         *   [4] target cool temp
         *   [5] cool temp change flag (bit 7)
         *   [6] fan speed cool
         *   [8] target heat temp
         *   [10] fan speed heat
         *   [11] heat fan change flag (bit 7)
         *   [15] fan mode
         */
        if (payload_len >= 1) state->power = (payload[0] & 0x01);
        if (payload_len >= 3) state->mode = decode_mode(payload[2]);
        if (payload_len >= 5) state->target_temp_cool = payload[4] * 10;
        if (payload_len >= 7) state->fan_mode_cool = decode_fan_speed(payload[6]);
        if (payload_len >= 9) state->target_temp_heat = payload[8] * 10;
        if (payload_len >= 11) state->fan_mode_heat = decode_fan_speed(payload[10]);
        state->data_valid = true;
        break;

    case PKT_TYPE_CTRL_3B:
        /*
         * 0x3B control exchange (FDYQ model M variant).
         * Similar to 0x38 but with zone support.
         *   [17] active zones
         *   [18] fan mode
         */
        if (payload_len >= 1) state->power = (payload[0] & 0x01);
        if (payload_len >= 3) state->mode = decode_mode(payload[2]);
        if (payload_len >= 5) state->target_temp_cool = payload[4] * 10;
        if (payload_len >= 7) state->fan_mode_cool = decode_fan_speed(payload[6]);
        if (payload_len >= 9) state->target_temp_heat = payload[8] * 10;
        if (payload_len >= 11) state->fan_mode_heat = decode_fan_speed(payload[10]);
        if (payload_len >= 18) state->active_zones = payload[17];
        state->data_valid = true;
        break;

    case PKT_TYPE_COUNTER_A3:
        /*
         * Counter data (operation hours, compressor starts, etc.)
         * Layout depends on counter sub-type in payload.
         */
        if (payload_len >= 8) {
            state->operation_hours = ((uint32_t)payload[0] << 24) |
                                    ((uint32_t)payload[1] << 16) |
                                    ((uint32_t)payload[2] << 8) |
                                    payload[3];
            state->compressor_starts = ((uint32_t)payload[4] << 24) |
                                       ((uint32_t)payload[5] << 16) |
                                       ((uint32_t)payload[6] << 8) |
                                       payload[7];
        }
        break;

    default:
        /* Other packet types — log at debug level */
        ESP_LOGD(TAG, "Unhandled packet type 0x%02X (len=%d)", type, pkt->length);
        break;
    }

    state->last_update_us = esp_timer_get_time();
    state->packet_count++;
}
