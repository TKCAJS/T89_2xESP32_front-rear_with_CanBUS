/**
 * DallasTemp.h
 * DS18B20 (1-Wire) radiator temperature for the sensor node.
 * Self-contained bit-banged driver — no OneWire / DallasTemperature libraries.
 * Non-blocking 1 Hz conversion. Plain functions, no classes.
 *
 * Why not the libraries (H523 build only; the F103 build still uses them):
 *  1. OneWire's ARDUINO_ARCH_STM32 backend implements its DIRECT_MODE_INPUT /
 *     DIRECT_MODE_OUTPUT macros as pin_function() -> HAL_GPIO_Init(), and
 *     read_bit() calls two of them *inside* the read slot. The DS18B20 holds
 *     read data valid for only 15 us after the falling edge; the two HAL calls
 *     plus the library's 3+10 us waits push the sample to ~17 us, past the
 *     window, so zeros come back as ones once a device is actually attached.
 *  2. DallasTemperature::getTempCByIndex() runs a full 64-bit ROM search on
 *     every call (~13 ms with interrupts masked in ~200 us chunks), and
 *     begin() walks the same search — which on a bus with corrupted bits
 *     wanders a bogus ROM tree.
 * Here the bus is a permanently open-drain pin driven by single-cycle BSRR
 * writes, and there is exactly one device on it so every transaction uses
 * SKIP ROM — no search anywhere. Scratchpad reads are CRC-checked, and every
 * loop is bounded, so a missing, shorted or noisy sensor costs ~1 ms once a
 * second and can never hang the node.
 *
 * PB0 — unchanged from the F103 build. Free GPIO on both parts (on this
 * variant PB0's only timer function is TIM1_CH2N, and TIM1 is unused; the
 * pump PWM is TIM3_CH4 on PB1). Wiring: DQ to PB0, 4.7k pull-up to the 3V3
 * rail, sensor VDD to 3V3 — not parasite-powered.
 */
#pragma once
#include <Arduino.h>

#define OW_PORT           GPIOB
#define OW_BIT            GPIO_PIN_0   // PB0
#define DALLAS_PERIOD_MS  1000         // request once per second
#define DALLAS_INVALID_C  (-127.0f)    // no/failed sensor — the DS18B20 world's
                                       // conventional "disconnected" value, and
                                       // far outside any real radiator temp

// Single device on the bus -> SKIP ROM addresses it without a ROM search.
#define OW_SKIP_ROM       0xCC
#define OW_CONVERT_T      0x44
#define OW_READ_SCRATCH   0xBE

static uint32_t s_lastDallasReq = 0;
static float    s_dallasTemp    = DALLAS_INVALID_C;
static bool     s_dallasValid   = false;
static bool     s_dallasPending = false;   // a conversion is in flight

// ── Bus primitives ────────────────────────────────────────────────────────────
// The pin stays an open-drain output for the whole session: pulling low and
// releasing are both single BSRR writes (~10 ns), and the external 4.7k does
// the pull-up. IDR still reads the true pin level in output mode, so no mode
// switching is needed anywhere in a bit slot — that is the whole fix.
#define OW_LOW()      (OW_PORT->BSRR = (uint32_t)OW_BIT << 16)
#define OW_RELEASE()  (OW_PORT->BSRR = (uint32_t)OW_BIT)
#define OW_READ()     ((OW_PORT->IDR & OW_BIT) != 0)

static void owInit() {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // ODR first (same idiom as fanInit) so the pin never drives the bus low as
    // it leaves the analog state gpio_blanket_init() left it in.
    OW_RELEASE();
    GPIO_InitTypeDef gpio = {};
    gpio.Pin   = OW_BIT;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;          // external 4.7k to 3V3
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OW_PORT, &gpio);
}

// Reset pulse + presence detect. False = bus stuck low (missing pull-up or a
// shorted cable) or nobody answered.
static bool owReset() {
    OW_RELEASE();
    // Bus must idle high before we start. Bounded at 250 us so a shorted line
    // can never stall the loop.
    for (uint8_t i = 0; i < 125 && !OW_READ(); i++) delayMicroseconds(2);
    if (!OW_READ()) return false;

    OW_LOW();
    delayMicroseconds(500);            // >=480 us reset pulse, no upper bound —
                                       // interrupts stay on, they can only
                                       // lengthen it
    // The device waits 15..60 us then pulls low for 60..240 us, so the bus is
    // guaranteed low only in the 60..75 us window: sample at 70, interrupts off.
    noInterrupts();
    OW_RELEASE();
    delayMicroseconds(70);
    bool present = !OW_READ();
    interrupts();
    delayMicroseconds(410);            // finish the reset slot
    return present;
}

static void owWriteBit(uint8_t v) {
    if (v) {
        // Write-1: low for 1..15 us. Tight, so mask interrupts for those 6 us.
        noInterrupts();
        OW_LOW();
        delayMicroseconds(6);
        OW_RELEASE();
        interrupts();
        delayMicroseconds(64);
    } else {
        // Write-0: low for >=60 us. The device samples 15..60 us in, so a late
        // release only makes the pulse longer than needed — interrupts stay
        // enabled for the long one, and 65 us keeps margin off the minimum.
        OW_LOW();
        delayMicroseconds(65);
        OW_RELEASE();
        delayMicroseconds(10);
    }
}

// The slot the library got wrong: the sample must land within 15 us of the
// falling edge. 2 + 8 us of BSRR-only work leaves ~5 us of margin for cable
// rise time.
static uint8_t owReadBit() {
    noInterrupts();
    OW_LOW();
    delayMicroseconds(2);
    OW_RELEASE();
    delayMicroseconds(8);
    uint8_t b = OW_READ() ? 1 : 0;
    interrupts();
    delayMicroseconds(55);             // finish the 65 us slot + recovery
    return b;
}

static void owWriteByte(uint8_t v) {           // LSB first
    for (uint8_t i = 0; i < 8; i++, v >>= 1) owWriteBit(v & 1);
}

static uint8_t owReadByte() {
    uint8_t v = 0;
    for (uint8_t i = 0; i < 8; i++) v |= owReadBit() << i;
    return v;
}

// Dallas/Maxim CRC-8 (x^8 + x^5 + x^4 + 1). Bitwise — 72 iterations for a
// scratchpad, cheaper than carrying a 256-byte table for one call site.
static uint8_t owCrc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        uint8_t byte = *data++;
        for (uint8_t i = 0; i < 8; i++, byte >>= 1) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
        }
    }
    return crc;
}

// ── DS18B20 transactions ──────────────────────────────────────────────────────

// Kick off a temperature conversion (~2 ms of bus time). The result is read
// back on the next 1 Hz tick, well past the 750 ms a 12-bit conversion needs.
static bool owStartConversion() {
    if (!owReset()) return false;
    owWriteByte(OW_SKIP_ROM);
    owWriteByte(OW_CONVERT_T);
    return true;
}

// Read the 9-byte scratchpad and convert (~7 ms of bus time).
static bool owReadTemp(float* out) {
    if (!owReset()) return false;
    owWriteByte(OW_SKIP_ROM);
    owWriteByte(OW_READ_SCRATCH);

    uint8_t s[9];
    for (uint8_t i = 0; i < 9; i++) s[i] = owReadByte();
    if (owCrc8(s, 9) != 0) return false;   // CRC over all 9 bytes (incl. the
                                           // stored CRC) is 0 when intact
    // Bits below the configured resolution are undefined; mask them per the
    // config byte (s[4] bits 6:5 = 9/10/11/12-bit). LSB is 1/16 C regardless.
    static const uint8_t RES_MASK[4] = { 0xF8, 0xFC, 0xFE, 0xFF };
    uint16_t raw = ((uint16_t)s[1] << 8) | (s[0] & RES_MASK[(s[4] >> 5) & 0x03]);
    *out = (int16_t)raw * 0.0625f;
    return true;
}

// ── Public interface ──────────────────────────────────────────────────────────

void dallasBegin() {
    owInit();
    s_dallasTemp    = DALLAS_INVALID_C;
    s_dallasValid   = false;
    s_dallasPending = owStartConversion();
    s_lastDallasReq = millis();
}

// Reads the previous conversion and kicks off the next, at 1 Hz. ~9 ms of bus
// time when a sensor is present, ~1 ms when it isn't. A sensor plugged in after
// boot needs no extra logic: the first tick after it appears starts a
// conversion, the tick after that reads it, so it self-heals within 2 s.
// Returns true when a fresh, CRC-valid reading was taken this call.
bool dallasUpdate(uint32_t now) {
    if (now - s_lastDallasReq < DALLAS_PERIOD_MS) return false;
    s_lastDallasReq = now;

    float t;
    s_dallasValid = s_dallasPending && owReadTemp(&t);
    s_dallasTemp  = s_dallasValid ? t : DALLAS_INVALID_C;

    s_dallasPending = owStartConversion();
    return s_dallasValid;
}

float dallasTempC() { return s_dallasTemp; }
bool  dallasValid() { return s_dallasValid; }
