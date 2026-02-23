
// Class definition for the renderer.  The renderer feeds the dotStar LED's column by column.
// It owns the front and back framebuffers.  It DMA's the pixel bytes after applying gamma
// filtering and doing logical to physical pixel location (rows/columns) translations.
// The framebuffer is logically row-0 at the bottom, row-47 at the top, column-0 at 
// the left, column 119 at the right.

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <globals.h>
#include <gamma.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>  // if we ever use GPIO.out_w1ts
#include <renderTypes.h>

class Renderer {
public:
  Renderer();  // Constructor

  void init();
  void freeBuffers();
  void sendColumn(uint16_t columnIndex);
  bool isBusy() const;
  void swapBuffers();
  uint8_t* getBackBuffer();
  void markBackBufferFilled();
  bool isBackBufferFilled();
  void clearFrameBuffer(FrameBuffer framebuffer);
  void setBrightness(uint8_t brightness);
  void setFramebufferOffset(int offset);
  int getFramebufferOffset();

private:
  int framebufferOffset;    // Shift where in the frame buffer we get the column to display.  Use this to scroll the image.
  void initSpi();
  void buildColumn(uint8_t *dst, uint8_t *colPtr);
  void startColumnDma(uint8_t *columnData);
  void pollDmaComplete();
  void ensureBuffersAllocated();

  // mapping of the menu brightness (0-10) to the dotStar five-bit brightness (0-31)
  uint8_t fiveBitBright; 

  spi_device_handle_t spi = nullptr;

  uint8_t *frontBuffer = nullptr;
  uint8_t *backBuffer  = nullptr;

  volatile bool dmaBusy = false;
  volatile bool backBufferFilled = false;
};
