#ifndef MATRIX_DISPLAY_H
#define MATRIX_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_NeoMatrix.h>
#include "GearFont.h"

// Matrix Configuration — four 8x8 panels tiled 2x2 into one 16x16 display.
#define MATRIX_PIN 16
#define PANEL_W  8
#define PANEL_H  8
#define TILES_X  2
#define TILES_Y  2
#define MATRIX_WIDTH  (PANEL_W * TILES_X)    // 16
#define MATRIX_HEIGHT (PANEL_H * TILES_Y)    // 16

// Panel/tile geometry, expressed in FRONT-view (LED side) coordinates.
//
// The panels have DI at bottom-left and DO at top-right *as seen from the back*,
// so viewed from the front each panel's first pixel is at its BOTTOM-RIGHT.
//
// Chain order, back view: bottom-left -> bottom-right -> top-left -> top-right.
// Mirrored to the front that is bottom-right first, tiles advancing right-to-left
// in rows, rows advancing upward, each row restarting on the right = TILE_PROGRESSIVE.
// Verified correct on hardware via the tile test pattern below.
//
// NEO_MATRIX_COLUMNS (the panel's internal scan) was determined empirically, not
// from the pad positions: ROWS and COLUMNS *both* put DI and DO on diagonally
// opposite corners, so the pads cannot distinguish them. ROWS was tried first and
// rendered each panel transposed. Same reason PROGRESSIVE is empirical rather than
// deduced -- on these clones the DIN pad is not necessarily wired to the LED
// physically nearest it, so the diagonal pad layout proves nothing on its own.
//
// If the image comes up wrong again after a re-wire, this is the line to edit:
// swap RIGHT<->LEFT / BOTTOM<->TOP for a flip, ROWS<->COLUMNS for a transpose,
// at the NEO_MATRIX_* level for within-panel faults or NEO_TILE_* for tile order.
#define MATRIX_LAYOUT_FLAGS ( \
    NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE + \
    NEO_TILE_BOTTOM   + NEO_TILE_RIGHT   + NEO_TILE_ROWS      + NEO_TILE_PROGRESSIVE)

// Diagnostic: set to 1 to freeze the display on a tile-identification pattern
// instead of running normally. Each logical quadrant gets one solid colour plus
// an asymmetric corner marker -- which physical panel shows which colour gives
// the NEO_TILE_* flags, and the marker's shape gives each panel's internal
// orientation. Set back to 0 once the mapping is correct.
#define MATRIX_TILE_TEST 0

// Layout: 2px-wide tachometer bar down the left edge, gear glyph fills the rest.
#define TACH_COLS  2
#define CORNER_SZ  2     // corner status blocks, scaled up from 1px on the old 8x8

class MatrixDisplay {
private:
    Adafruit_NeoMatrix* matrix;
    
    // Display state variables
    bool showShiftNotification;
    char shiftNotificationChar;
    unsigned long shiftNotificationStart;
    uint8_t notifR, notifG, notifB;
    unsigned long lastRainbowUpdate;

    // Configuration
    static const unsigned long SHIFT_NOTIFICATION_DURATION = 300;
    static const unsigned long CYCLE_TIME = 1000;
    static const unsigned long FLASH_DURATION = 70;
    static const unsigned long FLASH_GAP = 150;
    
    // External references needed
    bool* wifiEnabled;
    bool* canGearValid;
    bool* manualModeEnabled;

    const bool tileTest = (MATRIX_TILE_TEST != 0);

public:
    MatrixDisplay() : matrix(nullptr), showShiftNotification(false),
                     shiftNotificationChar(' '), shiftNotificationStart(0),
                     notifR(255), notifG(255), notifB(255),
                     lastRainbowUpdate(0),
                     wifiEnabled(nullptr), canGearValid(nullptr),
                     manualModeEnabled(nullptr) {}

    void begin(bool* wifiEnabledPtr, bool* canGearValidPtr, bool* manualModePtr = nullptr) {
        wifiEnabled = wifiEnabledPtr;
        canGearValid = canGearValidPtr;
        manualModeEnabled = manualModePtr;

        matrix = new Adafruit_NeoMatrix(PANEL_W, PANEL_H, TILES_X, TILES_Y, MATRIX_PIN,
            MATRIX_LAYOUT_FLAGS, NEO_GRB + NEO_KHZ800);

        matrix->begin();
        matrix->setBrightness(50);
        matrix->fillScreen(0);
        matrix->show();

        if (tileTest) {
            drawTileTestPattern();   // freezes the display; no live updates
        }
    }
    
    void update(const String& currentGearName) {
        if (tileTest) return;
        unsigned long currentMillis = millis();

        // Check if we should stop displaying the shift notification
        if (showShiftNotification && (currentMillis - shiftNotificationStart >= SHIFT_NOTIFICATION_DURATION)) {
            showShiftNotification = false;
        }
        
        // Update display every 50ms
        if (currentMillis - lastRainbowUpdate >= 50) {
            lastRainbowUpdate = currentMillis;
            
            matrix->fillScreen(0); // Clear screen (black)

            if (showShiftNotification) {
                drawCenteredChar(shiftNotificationChar, 0, MATRIX_WIDTH,
                                  matrix->Color(notifR, notifG, notifB));

            } else {
                // ALWAYS show current gear when not showing notification
                // Handle gear sensor disconnected state
                if (!(*canGearValid)) {
                    drawCenteredChar('?', 0, MATRIX_WIDTH, matrix->Color(255, 0, 0)); // Red text for error
                } else {
                    // Set color based on gear: Red for 1-6, Green for N
                    uint16_t color = (currentGearName == "N") ? matrix->Color(0, 255, 0)   // Green for Neutral
                                                               : matrix->Color(255, 0, 0);  // Red for gear numbers 1-6
                    drawCenteredChar(currentGearName[0], 0, MATRIX_WIDTH, color);
                }
            }
            
            // ADD MANUAL MODE INDICATOR - flash the bottom two rows amber
            addManualModeIndicator(currentMillis);
            
            // ADD HEARTBEAT - 2x2 block, top-right corner
            addHeartbeat(currentMillis);

            // ADD WIFI INDICATOR - flash the other three corners blue when WiFi on
            addWifiCornerFlash(currentMillis);

            matrix->show();
        }
    }
    
    void updateWithTachometer(const String& currentGearName, float currentRpm) {
        if (tileTest) return;
        unsigned long currentMillis = millis();

        // Check if we should stop displaying the shift notification
        if (showShiftNotification && (currentMillis - shiftNotificationStart >= SHIFT_NOTIFICATION_DURATION)) {
            showShiftNotification = false;
        }
        
        // Update display every 50ms
        if (currentMillis - lastRainbowUpdate >= 50) {
            lastRainbowUpdate = currentMillis;
            
            // ALWAYS update the tachometer (left columns) first
            updateTachometer(currentRpm);
            
            // Clear only the gear display area (not the tachometer columns)
            matrix->fillRect(TACH_COLS, 0, MATRIX_WIDTH - TACH_COLS, MATRIX_HEIGHT, 0);

            if (showShiftNotification) {
                // Display the shift notification (U/D) temporarily in center area
                drawCenteredChar(shiftNotificationChar, TACH_COLS, MATRIX_WIDTH - TACH_COLS,
                                  matrix->Color(notifR, notifG, notifB));

            } else {
                // ALWAYS show current gear when not showing notification
                // Handle gear sensor disconnected state
                if (!(*canGearValid)) {
                    drawCenteredChar('?', TACH_COLS, MATRIX_WIDTH - TACH_COLS, matrix->Color(255, 0, 0)); // Red text for error
                } else {
                    // Set color based on gear: Red for 1-6, Green for N
                    uint16_t color = (currentGearName == "N") ? matrix->Color(0, 255, 0)   // Green for Neutral
                                                               : matrix->Color(255, 0, 0);  // Red for gear numbers 1-6
                    drawCenteredChar(currentGearName[0], TACH_COLS, MATRIX_WIDTH - TACH_COLS, color);
                }
            }
            
            // ADD MANUAL MODE INDICATOR - flash the bottom two rows amber
            addManualModeIndicator(currentMillis);
            
            // ADD HEARTBEAT - 2x2 block, top-right corner
            addHeartbeat(currentMillis);

            // ADD WIFI INDICATOR - flash the other three corners blue when WiFi on
            addWifiCornerFlash(currentMillis);

            matrix->show();
        }
    }
    
    void displayShiftNotification(char notificationChar, uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) {
        shiftNotificationChar = notificationChar;
        notifR = r; notifG = g; notifB = b;
        showShiftNotification = true;
        shiftNotificationStart = millis();
    }

    void displayShiftLetter(char letter) {
        displayShiftNotification(letter);
    }
    
private:
    void updateTachometer(float rpm) {
        int ledsToLight = 0;
        uint16_t color = matrix->Color(0, 0, 0);
        bool shouldFlash = false;

        if (rpm >= 100) {
            if (rpm < 3000) {
                ledsToLight = 1;
                color = matrix->Color(0, 255, 0); // Green
            } else if (rpm < 5000) {
                ledsToLight = 2;
                color = matrix->Color(0, 255, 0); // Green
            } else if (rpm < 7500) {
                ledsToLight = 3;
                color = matrix->Color(0, 255, 0); // Green
            } else if (rpm < 9000) {
                ledsToLight = 4;
                color = matrix->Color(0, 255, 0); // Yellow
            } else if (rpm < 10500) {
                ledsToLight = 5;
                color = matrix->Color(255, 255, 0); // Yellow
            } else if (rpm < 12000) {
                ledsToLight = 6;
                color = matrix->Color(255, 165, 0); // Orange
            } else if (rpm < 13500) {
                ledsToLight = 7;
                color = matrix->Color(255, 0, 0); // Red
            } else {
                ledsToLight = 8;
                color = matrix->Color(255, 0, 0); // Red
                shouldFlash = true;
            }
        }
        
        // The RPM bands above still resolve to 0-8; the bar is 16 rows tall now,
        // so each band lights two rows. Keeps the tuned thresholds untouched.
        ledsToLight *= 2;

        // Clear the tachometer columns (x = 0..TACH_COLS-1)
        matrix->fillRect(0, 0, TACH_COLS, MATRIX_HEIGHT, 0);

        // Flash the whole bar off every 200ms at redline
        if (shouldFlash && (millis() / 200) % 2) return;

        // Light up the appropriate rows from bottom to top
        for (int i = 0; i < ledsToLight; i++) {
            matrix->drawFastHLine(0, (MATRIX_HEIGHT - 1) - i, TACH_COLS, color);
        }
    }
    
    void addManualModeIndicator(unsigned long currentMillis) {
        // Only show manual mode indicator if manual mode is enabled and reference is valid
        if (manualModeEnabled && *manualModeEnabled) {
            // Flash the bottom two rows amber at 500ms intervals
            bool flashOn = (currentMillis / 500) % 2;
            uint16_t amberColor = flashOn ? matrix->Color(255, 191, 0) : matrix->Color(0, 0, 0);

            matrix->fillRect(0, MATRIX_HEIGHT - 2, MATRIX_WIDTH, 2, amberColor);
        }
        // If manual mode is disabled or reference is null, bottom rows stay black (already cleared)
    }
    
    void addHeartbeat(unsigned long currentMillis) {
        // Copy the same heartbeat logic from the main LED
        unsigned long elapsed = currentMillis % CYCLE_TIME;
        
        if (elapsed < FLASH_DURATION ||
            (elapsed > FLASH_DURATION + FLASH_GAP &&
             elapsed < FLASH_DURATION * 2 + FLASH_GAP)) {
            // Heartbeat active - show red
            drawCorner(MATRIX_WIDTH - CORNER_SZ, 0, matrix->Color(255, 0, 0));
        } else {
            // Heartbeat background - blue intensity based on WiFi status
            int blueLevel = (*wifiEnabled) ? 255 : 5;  // Bright blue if WiFi on, dim if off
            drawCorner(MATRIX_WIDTH - CORNER_SZ, 0, matrix->Color(0, 10, blueLevel));
        }
    }

    void addWifiCornerFlash(unsigned long currentMillis) {
        // The top-right corner is the steady blue WiFi/heartbeat block. When WiFi is on,
        // flash the other three corners blue in sync so status reads from any corner.
        // Drawn only on the "on" phase so tach/gear content shows through between blinks.
        if (wifiEnabled && *wifiEnabled && ((currentMillis / 500) % 2)) {
            uint16_t blue = matrix->Color(0, 10, 255);
            drawCorner(0, 0, blue);                                              // top-left
            drawCorner(0, MATRIX_HEIGHT - CORNER_SZ, blue);                      // bottom-left
            drawCorner(MATRIX_WIDTH - CORNER_SZ, MATRIX_HEIGHT - CORNER_SZ, blue); // bottom-right
        }
    }

    void drawCorner(int16_t x, int16_t y, uint16_t color) {
        matrix->fillRect(x, y, CORNER_SZ, CORNER_SZ, color);
    }

    // Draws a GearFont glyph (see GearFont.h), centred within [originX,
    // originX + availWidth). Glyphs vary in width, so centre per-glyph
    // rather than assuming a fixed cell.
    void drawCenteredChar(char c, int16_t originX, int16_t availWidth, uint16_t color) {
        const GearGlyph* g = gearFontFind(c);
        if (!g) return;

        int16_t cx = originX + (availWidth - (int16_t)g->width) / 2;
        for (int16_t y = 0; y < GEAR_FONT_HEIGHT; y++) {
            uint16_t rowBits = g->rows[y];
            for (int16_t x = 0; x < g->width; x++) {
                if (rowBits & (1 << (g->width - 1 - x))) {
                    matrix->drawPixel(cx + x, y, color);
                }
            }
        }
    }

    // Tile-identification pattern. Paints each LOGICAL quadrant a distinct colour
    // and stamps an asymmetric black marker in that quadrant's logical top-left:
    // 3px across the top, 2px down the side.
    //
    // The marker MUST be asymmetric about the diagonal. A square notch (the first
    // version of this) is invariant under transposition, and so are the solid
    // colour blocks -- so a square notch cannot detect a transposed panel, which
    // is exactly the fault these clones turned out to have. 3-across/2-down reads
    // as 2-across/3-down when transposed.
    void drawTileTestPattern() {
        const int16_t hw = MATRIX_WIDTH / 2;
        const int16_t hh = MATRIX_HEIGHT / 2;
        const struct { int16_t x, y; uint8_t r, g, b; } quads[4] = {
            {  0,  0, 255,   0,   0 },   // logical TOP-LEFT     = RED
            { hw,  0,   0, 255,   0 },   // logical TOP-RIGHT    = GREEN
            {  0, hh,   0,   0, 255 },   // logical BOTTOM-LEFT  = BLUE
            { hw, hh, 255, 255, 255 },   // logical BOTTOM-RIGHT = WHITE
        };

        for (const auto& q : quads) {
            matrix->fillRect(q.x, q.y, hw, hh, matrix->Color(q.r, q.g, q.b));
            matrix->fillRect(q.x, q.y, 3, 1, 0);   // 3 across
            matrix->fillRect(q.x, q.y, 1, 2, 0);   // 2 down
        }
        matrix->show();
    }
};

#endif

// end of code