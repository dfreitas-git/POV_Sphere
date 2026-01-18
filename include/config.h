
#pragma once

// DotStar /  byte order
// NOTE: Many "APA102-compatible" strips (e.g. SK9822) may use GRB internally.
#define DOTSTAR_ORDER_GRB   0   // set to 0 for true APA102 (BGR)
#define GIMP_RGB565_LITTLE_ENDIAN  1  // Gimp ouputs rgb565 in little endian byte ordering