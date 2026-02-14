
#pragma once

#include <stdint.h>

// Minimal GLib type compatibility for GIMP C-source exports
// Gimp uses guint* syntax.  This aliases it to the usual uint* format
typedef uint8_t  guint8;
typedef uint16_t guint16;
typedef uint32_t guint;

