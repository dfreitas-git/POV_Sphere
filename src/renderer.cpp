
#include <renderer.h>


//#############################
// Initialize the framebuffers
//#############################
void Renderer::init() {
    initSpi();
    ensureBuffersAllocated();
}

//###########################################################
// Status about if we are in the middle of a DMA transaction
//###########################################################
bool Renderer::isBusy() const {
    return dmaBusy;
}

//#################################################
// Swap the backbuffer and frontbuffers atomically
//#################################################
void Renderer::swapBuffers() {
    if(!backBufferFilled) return;

    noInterrupts();
    uint8_t *tmp = frontBuffer;
    frontBuffer = backBuffer;
    backBuffer = tmp;
    interrupts();

    backBufferFilled = false;
}

//##############################################
// Once graphics is done filling the backbuffer 
//##############################################
void Renderer::markBackBufferFilled() {
    backBufferFilled = true;
}
//#############################
// Test if backbuffer is filed 
//#############################
bool Renderer::isBackBufferFilled() {
    if(backBufferFilled) {
       return(true);
    } else {
       return(false);
    }
}

// clear the specified framebuffer
void Renderer::clearFrameBuffer(uint8_t* frameBuffer) {
  memset(frameBuffer, 0, TOTAL_COLUMNS_RGB_BYTES);
}

//##################################################################
// Return a pointer to the backbuffer for graphics functions to use
//##################################################################
uint8_t* Renderer::getBackBuffer() {
    return backBuffer;
}

//###########################################
// DMA the new column data to the four rings
//###########################################
void Renderer::sendColumn(uint16_t columnIndex) {

    if(this->dmaBusy) return;

    uint8_t dotStarColumn[COLUMN_PAYLOAD];
    uint8_t *dst = dotStarColumn;

    int reversedColumn = COLUMNS - 1 - columnIndex;

    for(int ringIndex = 0; ringIndex < 4; ringIndex++) {

        uint8_t baseCol = ringIndex * COLS_PER_RING;

        uint8_t *colPtr =
            frontBuffer +
            (((baseCol + reversedColumn) % COLUMNS) * 3);

        buildColumn(dst, colPtr);

        digitalWrite(ringEnable[ringIndex], LOW);
        delayMicroseconds(1);

        startColumnDma(dst);

        while (this->dmaBusy) {
            pollDmaComplete();
        }

        digitalWrite(ringEnable[ringIndex], HIGH);
    }
}


//#################################################################################
// Allocate contiguous memory for the entire frame (COLUMNS * COLUMN_PAYLOAD)
//#################################################################################
void Renderer::ensureBuffersAllocated() {
  if (frontBuffer == nullptr) {
    frontBuffer = (uint8_t*)heap_caps_malloc(TOTAL_COLUMNS_RGB_BYTES, MALLOC_CAP_8BIT);
  }
  if (backBuffer == nullptr) {
    backBuffer  = (uint8_t*)heap_caps_malloc(TOTAL_COLUMNS_RGB_BYTES, MALLOC_CAP_8BIT);
  }
  if (!frontBuffer || !backBuffer) {
    Serial.println("ERROR: buffer allocation failed. Reduce buffer sizes or check memory.");
    while (1) { delay(1000); }
  }
}

//########################################################
// Free up framebuffer memory
//########################################################
void Renderer::freeBuffers() {
  if (frontBuffer) { free(frontBuffer); frontBuffer = nullptr; }
  if (backBuffer)  { free(backBuffer);  backBuffer  = nullptr; }
}

//############################################################
// Build one dotStar dotStar column (start+48*4+end) into dst
//############################################################
void Renderer::buildColumn(uint8_t *dst, uint8_t *colPtr) {
  int idx = 0;

  // Start frame (32 bits of zero)
  dst[idx++] = 0x00;
  dst[idx++] = 0x00;
  dst[idx++] = 0x00;
  dst[idx++] = 0x00;

  // Build dotStar data for each LED in the column
  for (int i = 0; i < LEDS_PER_COLUMN; ++i) {

    // Invert the rows since the POV_Sphere physically has row-0 at the top but we
    // fill the framebuffer assuming normal cartesian coords (0-+Y, 0+X from origin up, right)
    int physRow = LEDS_PER_COLUMN-1 - i;

    // Extract the rgb fields for this LED row
    uint8_t r = colPtr[physRow*COLUMNS*3];
    uint8_t g = colPtr[physRow*COLUMNS*3+1];
    uint8_t b = colPtr[physRow*COLUMNS*3+2];

    // Apply gamma correction
    r = gamma24[r];
    g = gamma26[g];
    b = gamma30[b];

    // Additional brightness control per color channel outside the dotStar built in global control
    r = (r * BRIGHTNESS_R) >> 8;
    g = (g * BRIGHTNESS_G) >> 8;
    b = (b * BRIGHTNESS_B) >> 8;

    // Start with brightness bits
    dst[idx++] = 0xE0 | fiveBitBright;   

    #if DOTSTAR_ORDER_GRB
        dst[idx++] = g;
        dst[idx++] = r;
        dst[idx++] = b;
    #else
        dst[idx++] = b;
        dst[idx++] = g;
        dst[idx++] = r;
    #endif
  }

  // End frame (enough 1s to latch last LEDs)
  dst[idx++] = 0xFF;
  dst[idx++] = 0xFF;
  dst[idx++] = 0xFF;
  dst[idx++] = 0xFF;
}

//###########################
//  SPI DMA setup 
//###########################
void Renderer::initSpi() {
  // configure SPI bus
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = PIN_SPI_MOSI;
  buscfg.miso_io_num = -1;
  buscfg.sclk_io_num = PIN_SPI_SCLK;
  buscfg.quadhd_io_num = -1;
  buscfg.quadwp_io_num = -1;
  // allow reasonably large transfers
  buscfg.max_transfer_sz = COLUMN_PAYLOAD * 2;

  // device config (dotStar doesn't use CS)
  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 8000000; // 8 MHz recommended (tune as desired)
  devcfg.mode = 0;
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 4;
  devcfg.flags = SPI_DEVICE_NO_DUMMY; // no dummy cycles

  esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    Serial.printf("spi_bus_initialize failed %d\n", ret);
    while (1) delay(1000);
  }

  ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  if (ret != ESP_OK) {
    Serial.printf("spi_bus_add_device failed %d\n", ret);
    while (1) delay(1000);
  }
  //Serial.println("SPI DMA initialized");
}

//###########################################################
// Start a DMA transfer of exactly one column (non-blocking).
//###########################################################
void Renderer::startColumnDma(uint8_t *columnData) {
  if (!this->spi) return;

  // Create a transaction on the stack and queue it
  spi_transaction_t *t = (spi_transaction_t*)heap_caps_malloc(sizeof(spi_transaction_t), MALLOC_CAP_DMA);
  if (!t) {
    Serial.println("Failed to allocate spi_transaction_t");
    return;
  }
  memset(t, 0, sizeof(spi_transaction_t));

  t->length = COLUMN_PAYLOAD * 8;      // bits
  t->tx_buffer = columnData;
  t->user = nullptr;

  // Mark that DMA is active
  this->dmaBusy = true;

  // Queue transaction (non-blocking)
  esp_err_t ret = spi_device_queue_trans(spi, t, portMAX_DELAY);
  if (ret != ESP_OK) {
    Serial.printf("spi_device_queue_trans error %d\n", ret);
    this->dmaBusy = false;
    free(t);
  }
}

//########################################################
// Poll for DMA completion and free transaction
//########################################################
void Renderer::pollDmaComplete() {
  if (!this->dmaBusy) return;

  spi_transaction_t *rtrans;
  esp_err_t ret = spi_device_get_trans_result(spi, &rtrans, 0); // timeout 0 => non-blocking
  if (ret == ESP_OK && rtrans != nullptr) {

    // Completed transaction
    // Free the transaction memory we allocated in startColumnDma
    heap_caps_free(rtrans);
    this->dmaBusy = false;
  }
  // if ret == ESP_ERR_TIMEOUT -> not finished yet; do nothing
}