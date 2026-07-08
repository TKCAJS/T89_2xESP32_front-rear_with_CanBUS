/**
 * SensorCan.h — F103 port
 * bxCAN (CAN1) init, send, receive and health monitoring for the sensor node.
 * TX: PB9  RX: PB8  (AFIO remap 2)
 *
 * Differences from the H562 FDCAN version:
 *  - bxCAN clocks straight off APB1 (36 MHz) — no kernel-clock mux to select.
 *  - 3 TX mailboxes instead of a 3-deep TX FIFO; TransmitFifoPriority=ENABLE
 *    keeps chronological order so the behaviour matches the FDCAN FIFO.
 *  - No GetProtocolStatus — bus health flags are read from the ESR register.
 *  - The RX FIFO0 interrupt shares its vector with USB low-priority
 *    (USB_LP_CAN1_RX0). USB is unused on this build, so the vector is ours.
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

static CAN_HandleTypeDef s_hcan;

static uint8_t s_seqOilPressure = 0;
static uint8_t s_seqWaterTemp   = 0;
static uint8_t s_seqTPS         = 0;
static uint8_t s_seqFuel1       = 0;
static uint8_t s_seqFuel2       = 0;
static uint8_t s_seqPumpStatus  = 0;
static uint8_t s_seqStatus      = 0;
static uint8_t s_seqHeartbeat   = 0;
static uint8_t s_seqRadiator    = 0;

// =============================================================================

bool canInit() {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    // Default CAN pins are PA11/PA12 (shared with USB). Remap 2 moves CAN1 to
    // RX=PB8 / TX=PB9 — closest match to the H562 board's PB8/PB7.
    __HAL_AFIO_REMAP_CAN1_2();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin   = GPIO_PIN_9;            // TX
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin  = GPIO_PIN_8;             // RX — pulled up so the line reads
    gpio.Mode = GPIO_MODE_INPUT;        // recessive if the transceiver is absent
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    s_hcan.Instance = CAN1;
    // 500 kbps @ 36 MHz APB1:
    //   prescaler=4 → 9 MHz, 18 TQ/bit (seg1=14, seg2=3), sample point 83%
    //   SJW capped at seg2 → 3 TQ
    s_hcan.Init.Prescaler            = 4;
    s_hcan.Init.Mode                 = CAN_MODE_NORMAL;
    s_hcan.Init.SyncJumpWidth        = CAN_SJW_3TQ;
    s_hcan.Init.TimeSeg1             = CAN_BS1_14TQ;
    s_hcan.Init.TimeSeg2             = CAN_BS2_3TQ;
    s_hcan.Init.TimeTriggeredMode    = DISABLE;
    s_hcan.Init.AutoBusOff           = DISABLE;   // manual recovery in canHealthPoll, as on the H562
    s_hcan.Init.AutoWakeUp           = DISABLE;
    s_hcan.Init.AutoRetransmission   = ENABLE;
    s_hcan.Init.ReceiveFifoLocked    = DISABLE;
    s_hcan.Init.TransmitFifoPriority = ENABLE;    // send in request order, like the FDCAN TX FIFO

    if (HAL_CAN_Init(&s_hcan) != HAL_OK) {
        Serial.println("[CAN] Init failed");
        return false;
    }

    // Accept everything into FIFO 0 (mask 0); extended-ID check happens in the
    // RX drain, matching the H562 filter behaviour.
    CAN_FilterTypeDef filter = {};
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    if (HAL_CAN_ConfigFilter(&s_hcan, &filter) != HAL_OK) {
        Serial.println("[CAN] Filter failed");
        return false;
    }

    if (HAL_CAN_Start(&s_hcan) != HAL_OK) {
        Serial.println("[CAN] Start failed");
        return false;
    }

    // Drain RX in the FIFO0 message-pending interrupt rather than polling. The
    // RX FIFO0 is fixed at 3 elements on bxCAN too, and the main loop blocks
    // for tens of ms on the SPI display write; at 50 Hz RPM the FIFO overruns
    // inside that window and the low-rate (on-change) gear frame is the
    // casualty. The ISR drains every frame the instant it arrives.
    if (HAL_CAN_ActivateNotification(&s_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Serial.println("[CAN] RX notify failed");
        return false;
    }
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    Serial.println("[CAN] bxCAN 500 Kbps OK");
    return true;
}

static bool canSend(uint32_t can_id, uint8_t seq, const uint8_t *payload, uint8_t payloadLen) {
    CAN_TxHeaderTypeDef hdr = {};
    hdr.ExtId              = can_id;
    hdr.IDE                = CAN_ID_EXT;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = 8;
    hdr.TransmitGlobalTime = DISABLE;

    uint8_t frame[8] = {};
    frame[0] = seq;
    frame[1] = g_nodeStatus;
    uint8_t len = (payloadLen > 6) ? 6 : payloadLen;
    memcpy(&frame[2], payload, len);

    // bxCAN has only 3 TX mailboxes. Each cycle the node queues 6-9 frames
    // back-to-back — far faster than they drain (~0.3 ms/frame at 500 kbps) —
    // so without waiting for a free mailbox the overflow frames (pump status,
    // TPS, fuel, heartbeat) would be silently dropped. Wait for a free slot,
    // bounded by a short guard so a stuck bus can't hang the loop.
    uint32_t start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&s_hcan) == 0) {
        if (HAL_GetTick() - start > 5) {
            g_nodeStatus |= NODE_STATUS_CAN_ERR;
            return false;
        }
    }

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&s_hcan, &hdr, frame, &mailbox) != HAL_OK) {
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        return false;
    }
    g_nodeStatus &= ~NODE_STATUS_CAN_ERR;
    return true;
}

// Drains RX FIFO0. Runs in ISR context (USB_LP_CAN1_RX0) — keep it
// allocation-free and Serial-free, and avoid read-modify-write on state the
// loop also writes (e.g. g_nodeStatus), which would race the TX path.
void canReceivePoll() {
    while (HAL_CAN_GetRxFifoFillLevel(&s_hcan, CAN_RX_FIFO0) > 0) {
        CAN_RxHeaderTypeDef hdr;
        uint8_t data[8] = {};
        if (HAL_CAN_GetRxMessage(&s_hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) break;
        if (hdr.IDE != CAN_ID_EXT) continue;

        // Any received message proves the bus is live
        if (!g_canReady && g_canHealth == CAN_HEALTH_NO_BUS) {
            g_canReady  = true;
            g_canHealth = CAN_HEALTH_OK;
        }

        if (hdr.ExtId == CAN_SENS_CMD_PUMP) {
            uint8_t duty = data[2];
            if (duty > 100) duty = 100;
            g_pumpDuty = duty;
        }
        else if (hdr.ExtId == CAN_SENS_CMD_TARGET_TEMP) {
            g_pumpTargetC = constrain(data[2], PUMP_TARGET_MIN, PUMP_TARGET_MAX);
        }
        else if (hdr.ExtId == CAN_MAIN_RPM) {
            g_rpm = data[2] | ((uint16_t)data[3] << 8);
        }
        else if (hdr.ExtId == CAN_REAR_GEAR_POS) {
            g_gear = data[2];
        }
    }
}

// Shared USB/CAN RX0 vector → HAL dispatch → RxFifo0 callback below.
extern "C" void USB_LP_CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&s_hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    canReceivePoll();
}

void canHealthPoll() {
    static uint8_t s_cleanPolls = 0;
    // bxCAN exposes bus health directly in ESR: BOFF (bus-off), EPVF (error
    // passive), EWGF (error warning) — same escalation the FDCAN protocol
    // status reported on the H562.
    uint32_t esr = CAN1->ESR;

    if (esr & CAN_ESR_BOFF) {
        s_cleanPolls  = 0;
        g_canHealth   = CAN_HEALTH_BUS_OFF;
        g_canReady    = false;
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        // Entering and leaving init mode kicks off the bus-off recovery
        // sequence (128 × 11 recessive bits), mirroring the H562 stop/start.
        HAL_CAN_Stop(&s_hcan);
        HAL_Delay(100);
        HAL_CAN_Start(&s_hcan);
        Serial.println("[CAN] Bus-off recovery");
        return;
    }

    if (esr & CAN_ESR_EPVF) {
        s_cleanPolls  = 0;
        g_canHealth   = CAN_HEALTH_NO_XCVR;
        g_canReady    = false;
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        return;
    }

    if (esr & CAN_ESR_EWGF) {
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
