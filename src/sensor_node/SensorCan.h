/**
 * SensorCan.h
 * FDCAN1 init, send, receive and health monitoring for the sensor node.
 * TX: PB7 (AF9)  RX: PB8 (AF9)  — fixed by PCB
 */

#pragma once
#include <Arduino.h>
#include "can_ids.h"
#include "SensorDisplay.h"  // for CanHealth enum

#define FDCAN_TX_PIN  GPIO_PIN_7
#define FDCAN_RX_PIN  GPIO_PIN_8

// Shared state — defined in main.cpp
extern uint8_t   g_nodeStatus;
extern bool      g_canReady;
extern CanHealth g_canHealth;
extern uint8_t   g_pumpDuty;

static FDCAN_HandleTypeDef s_hfdcan;

static uint8_t s_seqOilPressure = 0;
static uint8_t s_seqWaterTemp   = 0;
static uint8_t s_seqTPS         = 0;
static uint8_t s_seqFuel1       = 0;
static uint8_t s_seqFuel2       = 0;
static uint8_t s_seqPumpStatus  = 0;
static uint8_t s_seqStatus      = 0;
static uint8_t s_seqHeartbeat   = 0;

// =============================================================================

bool canInit() {
    // PB7 = FDCAN1_TX (AF9), PB8 = FDCAN1_RX (AF9)
    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = FDCAN_TX_PIN | FDCAN_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

    __HAL_RCC_FDCAN_CLK_ENABLE();

    s_hfdcan.Instance                  = FDCAN1;
    s_hfdcan.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    s_hfdcan.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    s_hfdcan.Init.Mode                 = FDCAN_MODE_NORMAL;
    s_hfdcan.Init.AutoRetransmission   = ENABLE;
    s_hfdcan.Init.TransmitPause        = DISABLE;
    s_hfdcan.Init.ProtocolException    = DISABLE;
    // 500 kbps @ 64 MHz FDCAN kernel clock:
    //   prescaler=8 → 8 MHz, 16 TQ/bit (seg1=11, seg2=4), sample point 75%
    s_hfdcan.Init.NominalPrescaler     = 8;
    s_hfdcan.Init.NominalSyncJumpWidth = 4;
    s_hfdcan.Init.NominalTimeSeg1      = 11;
    s_hfdcan.Init.NominalTimeSeg2      = 4;

    if (HAL_FDCAN_Init(&s_hfdcan) != HAL_OK) {
        Serial.println("[CAN] Init failed");
        return false;
    }

    // Accept all extended IDs into FIFO 0
    FDCAN_FilterTypeDef filter = {};
    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x00000000;
    filter.FilterID2    = 0x00000000;   // mask 0 → accept all
    if (HAL_FDCAN_ConfigFilter(&s_hfdcan, &filter) != HAL_OK) {
        Serial.println("[CAN] Filter failed");
        return false;
    }

    if (HAL_FDCAN_Start(&s_hfdcan) != HAL_OK) {
        Serial.println("[CAN] Start failed");
        return false;
    }

    Serial.println("[CAN] FDCAN1 500 Kbps OK");
    return true;
}

static bool canSend(uint32_t can_id, uint8_t seq, const uint8_t *payload, uint8_t payloadLen) {
    FDCAN_TxHeaderTypeDef hdr = {};
    hdr.Identifier          = can_id;
    hdr.IdType              = FDCAN_EXTENDED_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = FDCAN_DLC_BYTES_8;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;

    uint8_t frame[8] = {};
    frame[0] = seq;
    frame[1] = g_nodeStatus;
    uint8_t len = (payloadLen > 6) ? 6 : payloadLen;
    memcpy(&frame[2], payload, len);

    if (HAL_FDCAN_AddMessageToTxFifoQ(&s_hfdcan, &hdr, frame) != HAL_OK) {
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        return false;
    }
    g_nodeStatus &= ~NODE_STATUS_CAN_ERR;
    return true;
}

void canReceivePoll() {
    while (HAL_FDCAN_GetRxFifoFillLevel(&s_hfdcan, FDCAN_RX_FIFO0) > 0) {
        FDCAN_RxHeaderTypeDef hdr;
        uint8_t data[8] = {};
        if (HAL_FDCAN_GetRxMessage(&s_hfdcan, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK) break;
        if (hdr.IdType != FDCAN_EXTENDED_ID) continue;

        // Any received message proves the bus is live
        if (!g_canReady && g_canHealth == CAN_HEALTH_NO_BUS) {
            g_canReady    = true;
            g_canHealth   = CAN_HEALTH_OK;
            g_nodeStatus &= ~NODE_STATUS_CAN_ERR;
            Serial.println("[CAN] Bus recovered");
        }

        if (hdr.Identifier == CAN_SENS_CMD_PUMP) {
            uint8_t duty = data[2];
            if (duty > 100) duty = 100;
            g_pumpDuty = duty;
            Serial.printf("[CAN] Pump override: %u%%\n", duty);
        }
    }
}

void canHealthPoll() {
    static uint8_t s_cleanPolls = 0;
    FDCAN_ProtocolStatusTypeDef ps;

    if (HAL_FDCAN_GetProtocolStatus(&s_hfdcan, &ps) != HAL_OK) {
        s_cleanPolls  = 0;
        g_canHealth   = CAN_HEALTH_FAULT;
        g_canReady    = false;
        return;
    }

    if (ps.BusOff) {
        s_cleanPolls  = 0;
        g_canHealth   = CAN_HEALTH_BUS_OFF;
        g_canReady    = false;
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        HAL_FDCAN_Stop(&s_hfdcan);
        HAL_Delay(100);
        HAL_FDCAN_Start(&s_hfdcan);
        Serial.println("[CAN] Bus-off recovery");
        return;
    }

    if (ps.ErrorPassive) {
        s_cleanPolls  = 0;
        g_canHealth   = CAN_HEALTH_NO_XCVR;
        g_canReady    = false;
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        return;
    }

    if (ps.Warning) {
        s_cleanPolls  = 0;
        g_canHealth   = CAN_HEALTH_NO_BUS;
        g_canReady    = false;
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        return;
    }

    if (++s_cleanPolls >= 2) {
        s_cleanPolls  = 2;
        g_canHealth   = CAN_HEALTH_OK;
        g_canReady    = true;
        g_nodeStatus &= ~NODE_STATUS_CAN_ERR;
    }
}

// =============================================================================
// TX helpers
// Fixed-point encoding: °C×10 (int16_t), kPa×10 (int16_t), 0-10000 for 0-100%
// =============================================================================

void sendOilPressure(float kPa) {
    int16_t v = (int16_t)(kPa * 10.0f);
    uint8_t p[6] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    canSend(CAN_SENS_OIL_PRESSURE, s_seqOilPressure++, p, 6);
}

void sendWaterTemp(float degC) {
    int16_t v = (int16_t)(degC * 10.0f);
    uint8_t p[6] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    canSend(CAN_SENS_WATER_TEMP, s_seqWaterTemp++, p, 6);
}

void sendTPS(float pct) {
    uint16_t v = (uint16_t)(pct * 100.0f);
    uint8_t p[6] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    canSend(CAN_SENS_TPS, s_seqTPS++, p, 6);
}

void sendFuel1(float pct) {
    uint16_t v = (uint16_t)(pct * 100.0f);
    uint8_t p[6] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    canSend(CAN_SENS_FUEL_1, s_seqFuel1++, p, 6);
}

void sendFuel2(float pct) {
    uint16_t v = (uint16_t)(pct * 100.0f);
    uint8_t p[6] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    canSend(CAN_SENS_FUEL_2, s_seqFuel2++, p, 6);
}

void sendPumpStatus(uint8_t duty) {
    uint8_t p[6] = { duty };
    canSend(CAN_SENS_PUMP_STATUS, s_seqPumpStatus++, p, 6);
}

void sendSensStatus() {
    uint8_t p[6] = {};
    canSend(CAN_SENS_STATUS, s_seqStatus++, p, 6);
}

void sendHeartbeat() {
    uint8_t p[6] = { SOFTWARE_VERSION & 0xFF, (SOFTWARE_VERSION >> 8) & 0xFF };
    canSend(CAN_HB_SENSOR, s_seqHeartbeat++, p, 6);
}
