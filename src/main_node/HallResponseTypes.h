#ifndef HALL_RESPONSE_TYPES_H
#define HALL_RESPONSE_TYPES_H

// Hall sensor response curve configuration
// This enum is shared between multiple files, so it's defined once here
enum HallResponseCurve {
  HALL_LINEAR = 0,
  HALL_LOGARITHMIC = 1,
  HALL_EXPONENTIAL = 2,
  HALL_SMOOTH_STEP = 3,
  HALL_CUSTOM = 4,
  HALL_PIECEWISE = 5   // 3-zone map concentrating resolution on the biting zone
};

#endif

// end of code