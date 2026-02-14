
// Headers for the main core-1  rendering engine.  These are the functions that feed the LED's

#pragma once

#include <arduino.h>
#include <stdint.h>
#include <globals.h>
#include <gimp_compat.h>
#include <gamma.h>

#include <Wire.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>  // if we ever use GPIO.out_w1ts

extern spi_device_handle_t spi;

// Prototypes
void initSpi();
void buildColumn(uint8_t *dst, uint8_t *colPtr);
void startColumnDma(uint8_t *columnData);
void pollDmaComplete();
void updateColumnLEDs(uint16_t columnIndex);
void swapBuffersAtomic();
void ensureBuffersAllocated();
void freeBuffers();