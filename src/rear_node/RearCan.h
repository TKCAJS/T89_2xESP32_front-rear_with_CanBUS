/**
 * RearCan.h
 * CAN (TWAI) initialisation, send, receive and health monitoring
 * for the rear node.
 */

#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "can_ids.h"
#include "RearDisplay.h"
#include "GearShift.h"

#define GPIO_CAN_TX     4
#define GPIO_CAN_RX     5

// Shared state — defined in main.cpp, extern'd here
extern uint8_t   g_nodeStatus;
extern bool      g_canReady;
extern CanHealth g_canHealth;
extern RearDisplay g_display;

// Sequence counters
static uint8_t s_seqGearPos   = 0;
static uint8_t s_seqGearRaw   = 0;
static uint8_t s_seqStatus    = 0;
static uint8_t s_seqHeartbeat = 0;
static uint8_t s_seqShiftAck  = 0;

// Current gear — extern'd from main.cpp for shift command handling
extern uint8_t g_currentGear;

// =============================================================================

bool canInit() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)GPIO_CAN_TX,
        (gpio_num_t)GPIO_CAN_RX,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN] Driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] Start failed");
        return false;
    }
    Serial.println("[CAN] TWAI started at 500 Kbps");
    return true;
}

bool canSend(uint32_t can_id, uint8_t seq, uint8_t status,
             const uint8_t *payload, uint8_t payloadLen) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = can_id;
    msg.extd             = 1;
    msg.data_length_code = 8;
    msg.data[0]          = seq;
    msg.data[1]          = status;
    uint8_t len = (payloadLen > 6) ? 6 : payloadLen;
    memcpy(&msg.data[2], payload, len);

    esp_err_t result = twai_transmit(&msg, 0);  // non-blocking
    if (result != ESP_OK) {
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        return false;
    }
    g_nodeStatus &= ~NODE_STATUS_CAN_ERR;
    return true;
}

void canReceivePoll() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        if (!msg.extd) continue;
        // Any received message proves the bus is live — recover from NO_BUS
        if (!g_canReady && g_canHealth == CAN_HEALTH_NO_BUS) {
            g_canReady    = true;
            g_canHealth   = CAN_HEALTH_OK;
            g_nodeStatus &= ~NODE_STATUS_CAN_ERR;
            Serial.println("[CAN] Bus recovered — received message");
        }

        if (msg.identifier == CAN_REAR_CMD_SHIFT_UP || msg.identifier == CAN_REAR_CMD_SHIFT_DN) {
            // Payload (bytes 2-7):
            //   [2-3] shift_ms   uint16_t LE
            //   [4-5] ign_cut_ms uint16_t LE
            uint16_t shift_ms   = (uint16_t)msg.data[2] | ((uint16_t)msg.data[3] << 8);
            uint16_t ign_cut_ms = (uint16_t)msg.data[4] | ((uint16_t)msg.data[5] << 8);
            uint8_t  dir        = (msg.identifier == CAN_REAR_CMD_SHIFT_UP) ? SHIFT_UP : SHIFT_DOWN;

            Serial.printf("[CAN] Shift %s  shift_ms=%u  ign_cut_ms=%u\n",
                (dir == SHIFT_UP) ? "UP" : "DOWN", shift_ms, ign_cut_ms);

            g_display.setLastShift((ShiftDirection)dir);
            gearShiftRequest(dir, shift_ms, ign_cut_ms, g_currentGear);
        }
    }
}

// Distinguish no-transceiver from no-bus:
//   bus_error_count covers bit/stuff/form/CRC errors — these occur when the
//   physical layer is broken (no transceiver, TX not reflected back on RX).
//   tx_error_counter climbs from ACK errors even with a transceiver if no
//   other nodes are present, but bus_error_count stays low in that case.
static uint32_t s_prevBusErrCount = 0;

void canRestart() {
    Serial.println("[CAN] Restarting driver...");
    twai_stop();
    twai_driver_uninstall();
    delay(100);
    if (canInit()) {
        g_canReady    = true;
        g_canHealth   = CAN_HEALTH_NO_BUS;
        g_nodeStatus &= ~NODE_STATUS_CAN_ERR;
        s_prevBusErrCount = 0;
        Serial.println("[CAN] Driver restart OK");
    } else {
        g_canReady  = false;
        g_canHealth = CAN_HEALTH_FAULT;
        Serial.println("[CAN] Driver restart failed");
    }
}

void canHealthPoll() {
    twai_status_info_t info;
    static uint8_t s_cleanPolls = 0;

    if (twai_get_status_info(&info) != ESP_OK) {
        g_canHealth = CAN_HEALTH_FAULT;
        g_canReady  = false;
        return;
    }

    switch (info.state) {
        case TWAI_STATE_BUS_OFF:
        case TWAI_STATE_RECOVERING:
        case TWAI_STATE_STOPPED:
            g_canReady    = false;
            g_nodeStatus |= NODE_STATUS_CAN_ERR;
            canRestart();
            return;

        case TWAI_STATE_RUNNING: {
            uint32_t busErrDelta  = info.bus_error_count - s_prevBusErrCount;
            s_prevBusErrCount     = info.bus_error_count;

            if (busErrDelta > 15) {
                // Physical layer errors → no transceiver
                s_cleanPolls  = 0;
                g_canHealth   = CAN_HEALTH_NO_XCVR;
                g_canReady    = false;
                g_nodeStatus |= NODE_STATUS_CAN_ERR;
            } else if (info.tx_error_counter >= 24) {
                // Only ACK errors → transceiver present but no bus
                s_cleanPolls  = 0;
                g_canHealth   = CAN_HEALTH_NO_BUS;
                g_canReady    = false;
                g_nodeStatus |= NODE_STATUS_CAN_ERR;
            } else {
                // No errors — require 2 consecutive clean polls before declaring OK
                // (avoids false positive at boot before any TX has been attempted)
                if (++s_cleanPolls >= 2) {
                    g_canHealth   = CAN_HEALTH_OK;
                    g_canReady    = true;
                    g_nodeStatus &= ~NODE_STATUS_CAN_ERR;
                }
            }
            break;
        }

        default:
            g_canHealth = CAN_HEALTH_FAULT;
            g_canReady  = false;
            break;
    }
}

// =============================================================================
// TX HELPERS
// =============================================================================

void sendGearPos(uint8_t gear) {
    uint8_t payload[6] = { gear, 0, 0, 0, 0, 0 };
    canSend(CAN_REAR_GEAR_POS, s_seqGearPos++, g_nodeStatus, payload, 6);
}

void sendGearRaw(uint8_t rawPins, uint8_t gear) {
    uint8_t payload[6] = { rawPins, gear, 0, 0, 0, 0 };
    canSend(CAN_REAR_GEAR_RAW, s_seqGearRaw++, g_nodeStatus, payload, 6);
}

void sendStatus(uint8_t gear, uint8_t rawPins) {
    uint8_t payload[6] = { gear, rawPins, 0, 0, 0, 0 };
    canSend(CAN_REAR_STATUS, s_seqStatus++, g_nodeStatus, payload, 6);
}

void sendHeartbeat() {
    uint8_t payload[6] = { 0 };
    canSend(CAN_HB_REAR, s_seqHeartbeat++, g_nodeStatus, payload, 6);
}

// Shift ACK payload:
//   [2] direction     (SHIFT_UP / SHIFT_DOWN)
//   [3] expected gear
//   [4] actual gear
//   [5] result        (1=success, 0=failed)
void sendShiftAck(const ShiftResult &r) {
    uint8_t payload[6] = { r.dir, r.expectedGear, r.actualGear, r.success ? 1u : 0u, 0, 0 };
    canSend(CAN_REAR_ACK_COMPLETE, s_seqShiftAck++, g_nodeStatus, payload, 6);
}
