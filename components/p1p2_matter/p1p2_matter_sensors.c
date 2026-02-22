/*
 * P1P2 Matter Sensors — Temperature Measurement cluster management
 *
 * Maps temperature readings to Matter Temperature Measurement cluster (0x0402).
 * Multiple sub-endpoints for different temperature sources:
 *   - Outdoor temperature
 *   - Room/return temperature (also used by Thermostat LocalTemperature)
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "p1p2_matter.h"
#include "p1p2_matter_clusters.h"
#include "p1p2_protocol.h"

static const char *TAG = "matter_sens";

static int16_t prev_outdoor = 0x7FFF;
static int16_t prev_room = 0x7FFF;

void p1p2_matter_sensors_update(const p1p2_hvac_state_t *state)
{
    /* Matter temperatures: °C × 100 */
    int16_t outdoor = state->outdoor_temp * 10;  /* ×10 → ×100 */
    int16_t room = state->room_temp * 10;

    if (outdoor != prev_outdoor) {
        ESP_LOGD(TAG, "Outdoor temp: %d (%.1f°C)", outdoor, outdoor / 100.0);
        /*
         * TODO: esp_matter::attribute::update(
         *     EP_TEMP_SENSORS, CLUSTER_TEMP_MEASUREMENT,
         *     ATTR_MEASURED_VALUE, &outdoor);
         * This would be on a sub-endpoint for outdoor temperature.
         */
        prev_outdoor = outdoor;
    }

    if (room != prev_room) {
        ESP_LOGD(TAG, "Room temp: %d (%.1f°C)", room, room / 100.0);
        prev_room = room;
    }
}
