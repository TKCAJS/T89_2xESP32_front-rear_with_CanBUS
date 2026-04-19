#ifndef MATRIX_STARTUP_ANIMATION_H
#define MATRIX_STARTUP_ANIMATION_H

#include <Adafruit_NeoMatrix.h>
#include <math.h>

void runMatrixStartupAnimation(Adafruit_NeoMatrix* matrix) {
    matrix->fillScreen(0);
    matrix->show();

    // Phase 1: tachometer column sweeps 0 → redline (450ms)
    const uint32_t tachColors[8] = {
        matrix->Color(0,   255, 0),
        matrix->Color(0,   255, 0),
        matrix->Color(0,   255, 0),
        matrix->Color(180, 255, 0),
        matrix->Color(255, 200, 0),
        matrix->Color(255, 100, 0),
        matrix->Color(255, 0,   0),
        matrix->Color(255, 0,   0),
    };
    for (int lit = 0; lit <= 8; lit++) {
        for (int i = 0; i < 8; i++) {
            matrix->drawPixel(0, 7 - i, i < lit ? tachColors[i] : matrix->Color(0, 0, 0));
        }
        matrix->show();
        delay(55);
    }

    // Phase 2: columns 1–7 cascade fill bottom-up, staggered (560ms)
    // Colour shifts green → red across columns
    const uint32_t colColors[7] = {
        matrix->Color(0,   255, 0),
        matrix->Color(80,  255, 0),
        matrix->Color(160, 255, 0),
        matrix->Color(255, 200, 0),
        matrix->Color(255, 120, 0),
        matrix->Color(255, 40,  0),
        matrix->Color(255, 0,   0),
    };
    for (int frame = 0; frame < 9; frame++) {
        for (int col = 1; col < 8; col++) {
            int leds = constrain(frame - (col - 1), 0, 8);
            for (int row = 0; row < 8; row++) {
                bool on = row >= (8 - leds);
                matrix->drawPixel(col, row, on ? colColors[col - 1] : matrix->Color(0, 0, 0));
            }
        }
        matrix->show();
        delay(65);
    }

    // Phase 3: hold, then white flash (350ms)
    delay(150);
    matrix->fillScreen(matrix->Color(220, 220, 220));
    matrix->show();
    delay(120);
    matrix->fillScreen(matrix->Color(80, 80, 80));
    matrix->show();
    delay(80);

    // Phase 4: wipe columns off left-to-right (200ms)
    for (int col = 0; col < 8; col++) {
        for (int y = 0; y < 8; y++) {
            matrix->drawPixel(col, y, matrix->Color(0, 0, 0));
        }
        matrix->show();
        delay(25);
    }

    // Phase 5: red dot spirals outward from centre, leaving a dim trail (3000ms)
    // ~6 full revolutions while radius grows 0 → edge
    {
        const float cx = 3.5f, cy = 3.5f;
        const float ANGLE_STEP  = 0.50f;   // radians per frame
        const float RADIUS_STEP = 0.08f;   // pixels per frame → ~6.0 at frame 75
        const int   TOTAL_FRAMES = 75;     // 75 × 40ms = 3000ms

        bool trail[8][8] = {};
        float angle  = 0.0f;
        float radius = 0.0f;

        for (int frame = 0; frame < TOTAL_FRAMES; frame++) {

            // Draw trail pixels (dim red)
            matrix->fillScreen(0);
            for (int x = 0; x < 8; x++)
                for (int y = 0; y < 8; y++)
                    if (trail[x][y])
                        matrix->drawPixel(x, y, matrix->Color(120, 0, 0));

            // Compute dot position and mark trail
            int dx = (int)roundf(cx + cosf(angle) * radius);
            int dy = (int)roundf(cy + sinf(angle) * radius);
            if (dx >= 0 && dx < 8 && dy >= 0 && dy < 8) {
                trail[dx][dy] = true;
                matrix->drawPixel(dx, dy, matrix->Color(255, 0, 0)); // bright red dot
            }

            angle  += ANGLE_STEP;
            radius += RADIUS_STEP;

            matrix->show();
            delay(40);
        }

        matrix->fillScreen(0);
        matrix->show();
    }

    // Phase 6: T  8  9 in blue, one at a time
    {
        const char letters[] = { 'T', '8', '9' };
        matrix->setTextColor(matrix->Color(0, 80, 255));
        matrix->setTextSize(1);

        for (int i = 0; i < 3; i++) {
            matrix->fillScreen(0);
            matrix->setCursor(1, 0);
            matrix->print(letters[i]);
            matrix->show();
            delay(600);

            // Brief black gap between characters
            matrix->fillScreen(0);
            matrix->show();
            delay(100);
        }
    }
}

#endif // MATRIX_STARTUP_ANIMATION_H
