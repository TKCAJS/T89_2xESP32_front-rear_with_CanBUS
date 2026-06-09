#ifndef MAIN_CAN_H
#define MAIN_CAN_H

#include <Arduino.h>
#include <driver/twai.h>
#include "../../lib/can_ids/can_ids.h"

#define CAN_TX_PIN          17
#define CAN_RX_PIN          18
#define IGN_CUT_DEFAULT_MS  30
#define MAIN_NODE_VERSION   211

class MainCan {
private:
    bool    initialized;
    uint8_t txSeq;
    uint8_t _gear;
    bool    _gearValid;
    bool    _busLive;   // false until ≥1 frame received — gates periodic TX so
                        // main never floods (and bus-offs) into a not-yet-ready bus
    int16_t _radiatorTemp;   // °C × 10, from sensor node Dallas (CAN_SENS_RADIATOR_TEMP)

    bool install() {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
        twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        return twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
               twai_start() == ESP_OK;
    }

    void checkAndRecover() {
        twai_status_info_t info;
        if (twai_get_status_info(&info) != ESP_OK) return;

        if (info.state == TWAI_STATE_BUS_OFF ||
            info.state == TWAI_STATE_RECOVERING ||
            info.state == TWAI_STATE_STOPPED) {
            Serial.printf("CAN: state=%d — restarting\n", info.state);
            _busLive = false;   // re-confirm the bus is alive before resuming TX
            twai_stop();
            twai_driver_uninstall();
            delay(100);
            if (install()) {
                initialized = true;
                Serial.println("CAN: recovery OK");
            } else {
                Serial.println("CAN: recovery failed");
                initialized = false;
            }
        }
    }

public:
    MainCan() : initialized(false), txSeq(0), _gear(GEAR_UNKNOWN), _gearValid(false), _busLive(false), _radiatorTemp(0) {}

    uint8_t getGear() const { return _gear; }
    bool    isGearValid() const { return _gearValid; }
    float   getRadiatorTemp() const { return _radiatorTemp / 10.0f; }

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
        if (!install()) {
            Serial.println("CAN: init failed");
            return false;
        }
        initialized = true;
        Serial.println("CAN: initialized TX=GPIO17 RX=GPIO18 @ 500Kbps");
        return true;
    }

    // Send upshift command. ignCutMs=0 for neutral moves, IGN_CUT_DEFAULT_MS for gear shifts.
    void sendShiftUp(uint16_t shiftMs, uint16_t ignCutMs = IGN_CUT_DEFAULT_MS, uint8_t targetGear = GEAR_UNKNOWN) {
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
        msg.data[6] = targetGear;
        msg.data[7] = 0;
        if (twai_transmit(&msg, 0) != ESP_OK) {
            Serial.println("CAN: sendShiftUp TX failed");
        }
    }

    void sendShiftDown(uint16_t shiftMs, uint8_t targetGear = GEAR_UNKNOWN) {
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
        msg.data[6] = targetGear;
        msg.data[7] = 0;
        if (twai_transmit(&msg, 0) != ESP_OK) {
            Serial.println("CAN: sendShiftDown TX failed");
        }
    }

    void sendRpm(uint16_t rpm) {
        if (!initialized || !_busLive) return;
        twai_message_t msg = {};
        msg.extd             = 1;
        msg.identifier       = CAN_MAIN_RPM;
        msg.data_length_code = 8;
        msg.data[0] = txSeq++;
        msg.data[1] = NODE_STATUS_OK;
        msg.data[2] = rpm & 0xFF;
        msg.data[3] = (rpm >> 8) & 0xFF;
        if (twai_transmit(&msg, 0) != ESP_OK) {
            Serial.println("CAN: sendRpm TX failed");
        }
    }

    void sendShiftStack(uint8_t targetGear) {
        if (!initialized) return;
        twai_message_t msg = {};
        msg.extd             = 1;
        msg.identifier       = CAN_MAIN_SHIFT_STACK;
        msg.data_length_code = 8;
        msg.data[0] = txSeq++;
        msg.data[1] = NODE_STATUS_OK;
        msg.data[2] = targetGear;
        if (twai_transmit(&msg, 0) != ESP_OK) {
            Serial.println("CAN: sendShiftStack TX failed");
        }
    }

    void sendShiftStatus(bool manualModeActive) {
        if (!initialized || !_busLive) return;
        twai_message_t msg = {};
        msg.extd             = 1;
        msg.identifier       = CAN_MAIN_SHIFT_STATUS;
        msg.data_length_code = 8;
        msg.data[0] = txSeq++;
        msg.data[1] = NODE_STATUS_OK;
        msg.data[2] = manualModeActive ? 1 : 0;
        if (twai_transmit(&msg, 0) != ESP_OK) {
            Serial.println("CAN: sendShiftStatus TX failed");
        }
    }

    // Poll for incoming messages and check bus health — call from loop()
    void poll() {
        if (!initialized) return;

        static unsigned long lastHealthCheck = 0;
        static unsigned long lastHeartbeat   = 0;
        unsigned long now = millis();
        if (now - lastHealthCheck >= 1000 && now >= 2000) {
            checkAndRecover();
            lastHealthCheck = now;
        }
        if (_busLive && now - lastHeartbeat >= 1000) {
            twai_message_t hb = {};
            hb.extd             = 1;
            hb.identifier       = CAN_HB_MAIN;
            hb.data_length_code = 8;
            hb.data[0]          = txSeq++;
            hb.data[1]          = NODE_STATUS_OK;
            uint16_t ver        = MAIN_NODE_VERSION;
            hb.data[2]          = ver & 0xFF;
            hb.data[3]          = (ver >> 8) & 0xFF;
            twai_transmit(&hb, 0);
            lastHeartbeat = now;
        }

        twai_message_t msg;
        while (twai_receive(&msg, 0) == ESP_OK) {
            if (!msg.extd) continue;
            _busLive = true;   // heard a frame → bus is up and ACKing, TX is safe
            // Match on the FULL identifier (node + msgtype). Matching on msgtype
            // alone collides across nodes — e.g. CAN_SENS_OIL_PRESSURE shares
            // msgtype 0x01 with CAN_REAR_GEAR_POS and would corrupt the gear.
            if (msg.identifier == CAN_REAR_GEAR_POS) {
                _gear      = msg.data[2];
                _gearValid = (_gear <= GEAR_6 || _gear == GEAR_NEUTRAL);
            }
            else if (msg.identifier == CAN_SENS_RADIATOR_TEMP) {
                _radiatorTemp = msg.data[2] | ((int16_t)msg.data[3] << 8);
            }
            else if (msg.identifier == CAN_REAR_ACK_COMPLETE) {
                Serial.printf("CAN ACK_COMPLETE: dir=%d expected=%d actual=%d ok=%d\n",
                    msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
            }
        }
    }
};

#endif

// end of code
