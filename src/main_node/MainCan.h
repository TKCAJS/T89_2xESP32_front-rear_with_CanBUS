#ifndef MAIN_CAN_H
#define MAIN_CAN_H

#include <Arduino.h>
#include <driver/twai.h>
#include "../../lib/can_ids/can_ids.h"

#define CAN_TX_PIN          17
#define CAN_RX_PIN          18
#define IGN_CUT_DEFAULT_MS  30

class MainCan {
private:
    bool    initialized;
    uint8_t txSeq;
    uint8_t _gear;
    bool    _gearValid;

public:
    MainCan() : initialized(false), txSeq(0), _gear(GEAR_UNKNOWN), _gearValid(false) {}

    uint8_t getGear() const { return _gear; }
    bool    isGearValid() const { return _gearValid; }

    String getGearName() const {
        if (!_gearValid) return "?";
        switch (_gear) {
            case GEAR_NEUTRAL: return "N";
            case GEAR_1:       return "1";
            case GEAR_2:       return "2";
            case GEAR_3:       return "3";
            case GEAR_4:       return "4";
            case GEAR_5:       return "5";
            case GEAR_6:       return "6";
            default:           return "?";
        }
    }

    bool begin() {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
        twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
            Serial.println("CAN: driver install failed");
            return false;
        }
        if (twai_start() != ESP_OK) {
            Serial.println("CAN: start failed");
            return false;
        }
        initialized = true;
        Serial.println("CAN: initialized TX=GPIO17 RX=GPIO18 @ 500Kbps");
        return true;
    }

    // Send upshift command. ignCutMs=0 for neutral moves, IGN_CUT_DEFAULT_MS for gear shifts.
    void sendShiftUp(uint16_t shiftMs, uint16_t ignCutMs = IGN_CUT_DEFAULT_MS) {
        if (!initialized) return;
        twai_message_t msg = {};
        msg.extd             = 1;
        msg.identifier       = CAN_REAR_CMD_SHIFT_UP;
        msg.data_length_code = 8;
        msg.data[0] = txSeq++;
        msg.data[1] = NODE_STATUS_OK;
        msg.data[2] = shiftMs & 0xFF;
        msg.data[3] = (shiftMs >> 8) & 0xFF;
        msg.data[4] = ignCutMs & 0xFF;
        msg.data[5] = (ignCutMs >> 8) & 0xFF;
        msg.data[6] = 0;
        msg.data[7] = 0;
        if (twai_transmit(&msg, pdMS_TO_TICKS(5)) != ESP_OK) {
            Serial.println("CAN: sendShiftUp TX failed");
        }
    }

    void sendShiftDown(uint16_t shiftMs) {
        if (!initialized) return;
        twai_message_t msg = {};
        msg.extd             = 1;
        msg.identifier       = CAN_REAR_CMD_SHIFT_DN;
        msg.data_length_code = 8;
        msg.data[0] = txSeq++;
        msg.data[1] = NODE_STATUS_OK;
        msg.data[2] = shiftMs & 0xFF;
        msg.data[3] = (shiftMs >> 8) & 0xFF;
        msg.data[4] = 0;
        msg.data[5] = 0;
        msg.data[6] = 0;
        msg.data[7] = 0;
        if (twai_transmit(&msg, pdMS_TO_TICKS(5)) != ESP_OK) {
            Serial.println("CAN: sendShiftDown TX failed");
        }
    }

    // Poll for incoming messages — call from loop()
    void poll() {
        if (!initialized) return;
        twai_message_t msg;
        while (twai_receive(&msg, 0) == ESP_OK) {
            if (!msg.extd) continue;
            uint8_t msgType = CAN_ID_MSGTYPE(msg.identifier);
            switch (msgType) {
                case MSGTYPE_REAR_GEAR_POS:
                    _gear      = msg.data[2];
                    _gearValid = (_gear <= GEAR_6 || _gear == GEAR_NEUTRAL);
                    break;
                case MSGTYPE_REAR_ACK_COMPLETE:
                    Serial.printf("CAN ACK_COMPLETE: dir=%d expected=%d actual=%d ok=%d\n",
                        msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
                    break;
                default:
                    break;
            }
        }
    }
};

#endif

// end of code
