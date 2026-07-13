/**
 * SensorCan.h — H523 port
 * FDCAN1 (classic-frame mode) init, send, receive and health monitoring for
 * the sensor node. TX: PB7  RX: PB8  (closest match to the F103's PB9/PB8 —
 * FDCAN1 TX has no PB9 option on this chip; PB7 is the nearest free pin).
 *
 * Differences from the F103 bxCAN version:
 *  - FDCAN1 kernel clock is PLL1Q (50 MHz, see SystemClock_Config in
 *    main.cpp) — H5 has no plain-PCLK option for FDCAN like bxCAN did.
 *  - A single accept-all extended mask filter (mask=0) routes everything to
 *    RX FIFO0, same "filter in software by ID" approach as the F103 build —
 *    just via FDCAN_FilterTypeDef instead of CAN_FilterTypeDef.
 *  - GetProtocolStatus (BusOff/ErrorPassive/Warning flags) replaces the
 *    bxCAN ESR register read — same escalation, different API.
 *  - FDCAN1 has its own interrupt vector (FDCAN1_IT0), not a shared
 *    USB/CAN vector like the F103's USB_LP_CAN1_RX0.
 */

#pragma once
#include <Arduino.h>
#include "can_ids.h"
#include "SensorDisplay.h"  // for CanHealth enum
#include "PumpControl.h"    // for PUMP_TARGET_MIN/MAX clamp

// Shared state — defined in main.cpp.
// Fields touched by the RX ISR (canReceivePoll) are volatile so the main loop
// always observes fresh values.
extern uint8_t            g_nodeStatus;
extern volatile bool      g_canReady;
extern volatile CanHealth g_canHealth;
extern volatile uint8_t   g_pumpDuty;
extern volatile uint8_t   g_pumpTargetC;
extern volatile uint16_t  g_rpm;
extern volatile uint8_t   g_gear;

static FDCAN_HandleTypeDef s_hfdcan;

static uint8_t s_seqOilPressure = 0;
static uint8_t s_seqEngineTemp  = 0;
static uint8_t s_seqRadOut      = 0;
static uint8_t s_seqTPS         = 0;
static uint8_t s_seqFuel1       = 0;
static uint8_t s_seqFuel2       = 0;
static uint8_t s_seqPumpStatus  = 0;
static uint8_t s_seqStatus      = 0;
static uint8_t s_seqHeartbeat   = 0;
static uint8_t s_seqRadiator    = 0;

// =============================================================================

void canReceivePoll();   // forward decl — called from the RX ISR below

bool canInit() {
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = GPIO_PIN_7;        // TX
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin  = GPIO_PIN_8;             // RX — pulled up so the line reads
    gpio.Pull = GPIO_PULLUP;            // recessive if the transceiver is absent
    HAL_GPIO_Init(GPIOB, &gpio);

    // H5's FDCAN_InitTypeDef has no message-RAM element-count/size fields —
    // unlike the H723's driver, message RAM layout is fixed on this part,
    // not configurable per instance.
    s_hfdcan.Instance = FDCAN1;
    s_hfdcan.Init.ClockDivider       = FDCAN_CLOCK_DIV1;
    s_hfdcan.Init.FrameFormat        = FDCAN_FRAME_CLASSIC;
    s_hfdcan.Init.Mode               = FDCAN_MODE_NORMAL;
    s_hfdcan.Init.AutoRetransmission = ENABLE;
    s_hfdcan.Init.TransmitPause      = DISABLE;
    s_hfdcan.Init.ProtocolException  = DISABLE;
    // 50 MHz PLL1Q / prescaler 5 = 10 MHz, 20 tq/bit (1 + 15 + 4) = 500 kbit/s,
    // sample point 80%.
    s_hfdcan.Init.NominalPrescaler     = 5;
    s_hfdcan.Init.NominalSyncJumpWidth = 4;
    s_hfdcan.Init.NominalTimeSeg1      = 15;
    s_hfdcan.Init.NominalTimeSeg2      = 4;
    // Data phase unused (classic frames) — values just need to be legal
    s_hfdcan.Init.DataPrescaler     = 5;
    s_hfdcan.Init.DataSyncJumpWidth = 4;
    s_hfdcan.Init.DataTimeSeg1      = 15;
    s_hfdcan.Init.DataTimeSeg2      = 4;
    s_hfdcan.Init.StdFiltersNbr     = 0;
    s_hfdcan.Init.ExtFiltersNbr     = 1;
    s_hfdcan.Init.TxFifoQueueMode   = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&s_hfdcan) != HAL_OK) {
        Serial.println("[CAN] Init failed");
        return false;
    }

    // Accept everything (extended ID, mask 0) into FIFO0 — extended-ID check
    // and per-message routing happen in the RX drain, matching the F103
    // bxCAN filter behaviour (accept-all + software filter).
    FDCAN_FilterTypeDef filter = {};
    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x00000000;
    filter.FilterID2    = 0x00000000;   // mask = 0 -> match any ID
    if (HAL_FDCAN_ConfigFilter(&s_hfdcan, &filter) != HAL_OK) {
        Serial.println("[CAN] Filter failed");
        return false;
    }
    // Reject anything that isn't extended data (no standard IDs or remote
    // frames used anywhere in this system).
    if (HAL_FDCAN_ConfigGlobalFilter(&s_hfdcan, FDCAN_REJECT, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Serial.println("[CAN] Global filter failed");
        return false;
    }

    if (HAL_FDCAN_Start(&s_hfdcan) != HAL_OK) {
        Serial.println("[CAN] Start failed");
        return false;
    }

    // Drain RX in the FIFO0 message-pending interrupt rather than polling —
    // the loop blocks for tens of ms on the SPI display write, and at 50 Hz
    // RPM that window can overrun a shallow FIFO; the ISR drains every frame
    // the instant it arrives.
    if (HAL_FDCAN_ActivateNotification(&s_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Serial.println("[CAN] RX notify failed");
        return false;
    }
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

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

    uint8_t frame[8] = {};
    frame[0] = seq;
    frame[1] = g_nodeStatus;
    uint8_t len = (payloadLen > 6) ? 6 : payloadLen;
    memcpy(&frame[2], payload, len);

    // H5's TX FIFO/Queue is fixed at 3 elements (SRAMCAN_TFQ_NBR in the HAL —
    // not configurable like the H723's, and coincidentally the same depth as
    // the F103's 3 bxCAN mailboxes). Each cycle the node queues 6-9 frames
    // back-to-back — far faster than they drain (~0.16 ms/frame at
    // 500 kbit/s) — so without waiting for a free slot the overflow frames
    // (pump status, TPS, fuel, heartbeat) would be silently dropped. Wait for
    // room, bounded by a short guard so a stuck bus can't hang the loop.
    uint32_t start = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&s_hfdcan) == 0) {
        if (HAL_GetTick() - start > 5) {
            g_nodeStatus |= NODE_STATUS_CAN_ERR;
            return false;
        }
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&s_hfdcan, &hdr, frame) != HAL_OK) {
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        return false;
    }
    g_nodeStatus &= ~NODE_STATUS_CAN_ERR;
    return true;
}

// Drains RX FIFO0. Runs in ISR context (FDCAN1_IT0) — keep it
// allocation-free and Serial-free, and avoid read-modify-write on state the
// loop also writes (e.g. g_nodeStatus), which would race the TX path.
void canReceivePoll() {
    while (HAL_FDCAN_GetRxFifoFillLevel(&s_hfdcan, FDCAN_RX_FIFO0) > 0) {
        FDCAN_RxHeaderTypeDef hdr;
        uint8_t data[8] = {};
        if (HAL_FDCAN_GetRxMessage(&s_hfdcan, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK) break;
        if (hdr.IdType != FDCAN_EXTENDED_ID) continue;

        // Any received message proves the bus is live
        if (!g_canReady && g_canHealth == CAN_HEALTH_NO_BUS) {
            g_canReady  = true;
            g_canHealth = CAN_HEALTH_OK;
        }

        if (hdr.Identifier == CAN_SENS_CMD_PUMP) {
            uint8_t duty = data[2];
            if (duty > 100) duty = 100;
            g_pumpDuty = duty;
        }
        else if (hdr.Identifier == CAN_SENS_CMD_TARGET_TEMP) {
            g_pumpTargetC = constrain(data[2], PUMP_TARGET_MIN, PUMP_TARGET_MAX);
        }
        else if (hdr.Identifier == CAN_MAIN_RPM) {
            g_rpm = data[2] | ((uint16_t)data[3] << 8);
        }
        else if (hdr.Identifier == CAN_REAR_GEAR_POS) {
            g_gear = data[2];
        }
    }
}

extern "C" void FDCAN1_IT0_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(&s_hfdcan);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) canReceivePoll();
}

void canHealthPoll() {
    static uint8_t s_cleanPolls = 0;
    FDCAN_ProtocolStatusTypeDef psr;
    if (HAL_FDCAN_GetProtocolStatus(&s_hfdcan, &psr) != HAL_OK) return;

    if (psr.BusOff) {
        s_cleanPolls  = 0;
        g_canHealth   = CAN_HEALTH_BUS_OFF;
        g_canReady    = false;
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        // Entering and leaving init mode kicks off the bus-off recovery
        // sequence, mirroring the F103 stop/start.
        HAL_FDCAN_Stop(&s_hfdcan);
        HAL_Delay(100);
        HAL_FDCAN_Start(&s_hfdcan);
        Serial.println("[CAN] Bus-off recovery");
        return;
    }

    if (psr.ErrorPassive) {
        s_cleanPolls  = 0;
        g_canHealth   = CAN_HEALTH_NO_XCVR;
        g_canReady    = false;
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        return;
    }

    if (psr.Warning) {
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

void sendEngineTemp(float degC) {
    int16_t v = (int16_t)(degC * 10.0f);
    uint8_t p[6] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    canSend(CAN_SENS_ENGINE_TEMP, s_seqEngineTemp++, p, 6);
}

void sendRadOutTemp(float degC) {
    int16_t v = (int16_t)(degC * 10.0f);
    uint8_t p[6] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    canSend(CAN_SENS_RAD_OUT_TEMP, s_seqRadOut++, p, 6);
}

void sendRadiatorTemp(float degC) {
    int16_t v = (int16_t)(degC * 10.0f);
    uint8_t p[6] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    canSend(CAN_SENS_RADIATOR_TEMP, s_seqRadiator++, p, 6);
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
