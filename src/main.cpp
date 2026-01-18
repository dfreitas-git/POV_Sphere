// POV_DotStar_DMA.ino
// ESP32 Arduino sketch: SK9822 (DotStar) driven with DMA, double-buffered,
// 4x-per-revolution, and one SPI channel serially multiplexing the DMA streams
// to the dotStar strips.
//
// Hardware assumptions:
// - ESP32 using core-0 for the slow stuff: UI, menu, rotary-switch inputs, motor control.  
//               core-1 for the fast stuff: framebuffer reading, DMA, LED strip rendering.
// - SK9822 / DotStar LED chains: 4 rings, each 48 LEDs (total framebuffer 120x48).
// - A SN74AHCT125 tri-state bus buffer to switch MOSI to one of 4 rings.
// - AS5600 magnetic encoder is used to monitor the Sphere shaft angle to adjust motorRPM, synchronize
//   the core-1 PLL.
// - Using a 128x64 OLED and rotary-encoder/switch for a simple menu system for user controls. 
//
// Image creation:
// - Images are imported into Gimp then scaled to 120x48 and exported using the File->"Export as C-source" format.
//   The only options to set in the export form are "Use GLib types (guint8)" and "Save as RGB565 (16-bit)".
// - Edit the file include/images.h and add the file that was exported.  Add a wrapper for it:
//  const Image ImageNameWrap = {
//    .name = "ImageName",
//    .width = ImageName.width,
//    .height = ImageName.height,
//    .bytes_per_pixel = ImageName.bytes_per_pixel,
//    .pixel_data = ImageName.pixel_data
//  };
//
// - Finally, add an entry into the enum ImageID {} list and the imageTable[IMG_COUNT] array to the images.h file.
//
// Image generation:
// - Functions with the name fillBB_<name> are algorithmic generation of patterns.  They are time based and compute
//   the colors around the globe in terms of a periodic frequency, phase, cycle all derived from the absolute millis() time.
//   Then we scan the rows/columns of the framebuffer, loading the computed color into the available LED locations.  Think of 
//   it as the functions can compute color for any spot on the Sphere,  the framebuffer loads its more sparsely available 
//   points by reading the sphere colors at those points.
// - Edit the images.h file to add wrappers for each of the animation functions.  The functions themselves are in the images.cpp file.
//
// Controls
// -  A rotary/switch encoder is the input device.  A 128x64 OLED is the menu display.  Rotate the knob to select
//    a menu item.  Click to select the command.  Some commands allow you to enter a value by rotating the knob.
//    Some commands execute with another click.  Double-click to exit a command or return from a sub-menu.
//
// Power
// - The POV_Sphere is powered from a 12v battery (3s2p lithium).  There is a charge jack on the battery case.
//   A BMS is built in so you only need to supply 12.6-13v and it will limit the charge to the batteries.  
//
// - *** NOTE: Do not turn on the power switch or run the Sphere while charging the batteries.
//

// The majority of this code was written by colaborating with chatGPT.  I modified by add menu specifics, hardware specifics, etc.
// dlf 12/28/2025

#include <Arduino.h>
#include <stdint.h>

#include <config.h>
#include <globals.h>
#include <graphicsFunctions.h>
#include <gimp_compat.h>
#include <gamma.h>
#include <images.h>
#include <UI.h>
#include <motor.h>

#include <Wire.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>  // if we ever use GPIO.out_w1ts

// Prototypes
struct MenuItem;
void initSpi();
void buildColumn(uint8_t *dst, uint8_t *colPtr);
void startColumnDma(uint8_t *columnData);
void pollDmaComplete();
void updateColumnLEDs(uint16_t columnIndex);
void swapBuffersAtomic();
void ensureBuffersAllocated();
void freeBuffers();
void uiTask(void* parameter) ;

// For generating sync pulses to measure timing using a scope
#define SYNC_PIN 26
#define SYNC_MASK (1UL << SYNC_PIN)

// Semaphore to be sure setup is 100% complete before we start executing loop in core-0
SemaphoreHandle_t initDone;

// For driving a sync-pulse high and low
inline void IRAM_ATTR sync_high() {
  REG_WRITE(GPIO_OUT_W1TS_REG, SYNC_MASK);
}
inline void IRAM_ATTR sync_low() {
  REG_WRITE(GPIO_OUT_W1TC_REG, SYNC_MASK);
}

// ###################
//   Objects used
// ###################

// SPI and DMA objects
spi_device_handle_t spi = nullptr;

// for the core-0 task
TaskHandle_t uiTaskHandle;


//#########################################
// Main Setup Code
//#########################################
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Create a semaphore for core-0 to use to be sure setup is complete before starting core-0 loop
  initDone = xSemaphoreCreateBinary();

  // Pin setup
  pinMode(RING0_ENB, OUTPUT);
  pinMode(RING1_ENB, OUTPUT);
  pinMode(RING2_ENB, OUTPUT);
  pinMode(RING3_ENB, OUTPUT);

  // Sync pin for scope measurements
  pinMode(SYNC_PIN, OUTPUT);

  //Set up OLED on 2nd I2C bus
  Wire1.begin(SDA_OLED, SCL_OLED, 400000); // SDA, SCL, clock

  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed");
    while (true);
  }
  oled.clearDisplay();

  // Set up the rotary encoder/switch
  encoderTicker.attach_ms(1, encoderService);

  // Fill the initial menu 
  buildMenu();

  // Set up pwm
  setupMotor();

  // Uses default i2c interface to AS5600 to read angle
  if (!as5600.begin()) {
    Serial.println("Could not find AS5600 sensor, check wiring!");
    while (1) {
      delay(10);
    }
  }
  Wire.setClock(800000);  // Speed up I2c to AS5600

  // Allocate buffers
  ensureBuffersAllocated();

  // Initialize SPI with DMA
  initSpi();

  // Small stabilization time
  delay(10);

  // Get the angle the motor shaft is starting at and initialize the core-1 angle-accumulator/calculator
  measuredAngle = as5600.getRawAngle(); 
  angle_q = measuredAngle; 

  // For the core-0 angle measurement update
  lastMeasuredTime = millis();  //Used by core-0
  lastScrollTime = lastMeasuredTime;  // Used for framebuffer scrolling when rotating the image around the Sphere

  // Used by core-1 when calculating it's PLL angle
  lastAngleTime = micros(); 

  // Initialize the column number based on where the shaft is sitting
  // Do multiply before divide to maintain precision
  columnIndex = (uint32_t(measuredAngle) * COLUMNS) / AS5600_COUNTS;

  // Need to set where the next column will start
  constexpr uint32_t base_step = AS5600_COUNTS / COLUMNS;
  nextColumnAngle = (columnIndex + 1) * base_step;

  // Create tase to run on core-0 for the UI code
  xTaskCreatePinnedToCore(
    uiTask,
    "UI Task",
    4096,
    nullptr,
    1,
    &uiTaskHandle,
    0   //  Core 0
  );

  // We are done with setup.  Release the semaphore
  xSemaphoreGive(initDone);   // LAST LINE
}

//###################################################################
// core-0 loop (where we will run all the UI and SDCard reading)
// Task for core-0 
//####################################################################
void uiTask(void* parameter) {

  // Be sure the setup section is complete before we start the core-0 loop
  xSemaphoreTake(initDone, portMAX_DELAY);

  for (;;) {

    //sync_high();  // Debug rising edge to measure timing on a scope
    //sync_low();  // Debug falling edge to measure timing on a scope

    // Periodically measure the shaft angle for motor rpm/PID control and core-1 PLL locking
    uint32_t now = micros();
    int angleDelta;
    if (now - lastMeasuredTime >= samplePeriod_us) {

      // Go do a shaft angle measurement so core-1 angle-calculating "pll" can lock to it. 
      measuredAngle = as5600.getRawAngle();

      // When we get a new actual shaft-angle measurement, use it to adjust our core-1 computed angle to eliminate any drift
      phase_error = measuredAngle - angle_q;

      // Takes care of case where we cross the 360 degree back to 0 degree boundary
      if (phase_error > 2048)  phase_error -= AS5600_COUNTS;
      if (phase_error < -2048) phase_error += AS5600_COUNTS;

      // Angle error less than this and we won't apply any more correction to the VCO
      if (abs(long(phase_error)) < PHASE_DEADBAND){
        phase_error = 0;
      }

      // Small proportional-only correction in frequency to keep angle in sync 
      omega_trim = (phase_error << OMEGA_SHIFT) / OMEGA_TRIM_PERIOD;

      // Limit the trim amount
      omega_trim = constrain(omega_trim, -MAX_TRIM, MAX_TRIM);

      // Check the motor speed
      angleDelta = measuredAngle - lastAngle;
  
      // Takes care of case where we cross the 360 degree back to 0 degree boundary
      if (angleDelta > 2048)  angleDelta -= AS5600_COUNTS;
      if (angleDelta < -2048) angleDelta += AS5600_COUNTS;
  
      // Compute rpm by measuring the angle Delta between polling
      float dt_s = (now - lastMeasuredTime) * 1e-6;
      motorRPM = (angleDelta * 60.0) / ((AS5600_COUNTS * 1.0) * dt_s);

      // Compute the shaft speed that will be used by core-1 to calculate the shaft angle
      // omega_ff is in angle-counts/us  (scaled up by OMEGA_SHIFT for integer math.)
      omega_ff = int32_t(((motorRPM * pow(2,OMEGA_SHIFT)/ 60) * AS5600_COUNTS) / 1000000);

      /*
      if(motorOnOffFlag) {
        Serial.printf("RPM: %.1f  now: %d   lastMeasuredTime: %d   dt_s: %.4f  angleDelta: %d  measuredAngle: %d\n", motorRPM, now, lastMeasuredTime,dt_s, angleDelta, measuredAngle);
      }
      */

      lastAngle = measuredAngle;
      lastMeasuredTime = now;
    }
  
    // Rotates the image by shifting the index into the framebuffer
    unsigned long curMillis = millis();
    if(scrollOnOffFlag &&  curMillis - lastScrollTime > SCROLL_UPDATE_TIME) {
      framebufferOffset--;  // Shift where in the frame buffer we get the column to display.  Use this to scroll the image.
      if(framebufferOffset < 0) {
        framebufferOffset = COLUMNS - 1;
      }
      lastScrollTime=curMillis;
    }
    
    //  PID speed regulation
    if (curMillis - lastPidMs > PID_UPDATE_TIME) {
      float pwm = updatePID(motorRPM, targetRPM);
  
      // See if the user wants the motor off
      if(motorOnOffFlag == 0) {
        ledcWrite(0, 0);
      } else {
        ledcWrite(0, (int)pwm);
      }
    }
    // Now go check the encoder/switch for user input
    int encoderDelta = encoder.getValue();
    if (encoderDelta != 0) {
      handleRotation(encoderDelta);
    }

    ClickEncoder::Button b = encoder.getButton();
    if (b == ClickEncoder::Clicked) {
      handleClick();
    }
    if (b == ClickEncoder::DoubleClicked) {
      handleDoubleClick();
    }

    // Update the OLED
    updateBlink();
    drawMenu();

    // Global brightness: 0b111xxxxx (5-bit current control)
    // Read the brightness setting (can be changed in the OLED menu)  Map to 0-1F
    fiveBitBright = map(brightness,0,10,0,31);

    // Load the backBuffer with the next frame to display
    if (!backBufferFilled) {
      imageTable[imageToDisplayIndex]->functionPtr();
      backBufferFilled = true;
      lastAnimateTime = millis();
    }
  }
}


//#########################################################################
// Main Loop for core-1. Core-1 will run the critical POV data DMA loop.
//#########################################################################
void loop() {

  // Make a local copy of the shaft speed that core-0 measured.
  core_1_omega_ff = omega_ff;      // local copies used by core-1.
  core_1_omega_trim = omega_trim;     

  // Now compute the angle_q for the current time (i.e. what the Sphere angle currently is)
  computeCurrentAngle();   

  uint16_t adjustedAngle = angle_q % AS5600_COUNTS;  // modulo to wrap result in case of overflow

  int32_t triggerPoint = adjustedAngle - nextColumnAngle;
  if(triggerPoint < -2048) triggerPoint += AS5600_COUNTS;
  if(triggerPoint >  2048) triggerPoint -= AS5600_COUNTS;


  // Once the current angle has reached the next column, trigger a DMA transfer if the backBuffer is ready with a new frame
  if(triggerPoint >= 0) {

    // As soon as core-0 filles the backBuffer, swap it into the frontBuffer and start updating the Sphere
    if(backBufferFilled) {

      // Point the frontbuffer to the new data
      swapBuffersAtomic();

     // Reset flag so core-0 can start filling the back buffer again
     backBufferFilled = false;
    }

    // Check to see how many columns the shaft advanced.  Should usually be 1, but if there was some CPU delay the shaft may have advanced further
    uint32_t columnsAdvanced = (triggerPoint * COLUMNS / AS5600_COUNTS) + 1;

    columnIndex = (columnIndex + columnsAdvanced) % COLUMNS;
    nextColumnAngle = (columnIndex + 1) * (AS5600_COUNTS / COLUMNS);

    // Take care of angle wrapping from 360->0
    if (nextColumnAngle >= AS5600_COUNTS) {
      nextColumnAngle -= AS5600_COUNTS;
    }
  
    // Go update the strips. 
    if(!dmaBusy) {
      updateColumnLEDs((columnIndex + framebufferOffset) % COLUMNS);
    }
  }
}

// #############################################################
//  Functions
// #############################################################


// #########################################
// DMA the new column data to the four rings
// #########################################
void updateColumnLEDs(uint16_t columnIndex) {

  uint8_t dotStarColumn[COLUMN_PAYLOAD];  // Use to hold transformed column data (rgb565->rgb888, gamma, brightness)
  uint8_t *dst = dotStarColumn;

  // Need to reverse the column index since the sphere is rotating clockwise which means it's 
  // painting right to left from the framebuffer (i.e. highest index to lowest)

  int reversedColumn = COLUMNS - 1 - columnIndex;

  // Start DMA for current column.  For each column, stream out the four rings led data
  for(int ringIndex = 0; ringIndex < 4; ringIndex++) {
    uint8_t baseCol = ringIndex * COLS_PER_RING;
    uint8_t *colPtr;

    colPtr = frontBuffer + (((baseCol + reversedColumn) % COLUMNS) * 3);  // modulo 120 so we wrap when not starting at col-0.  Multiply by 3-bytes to step across rgb fields

    // Transform column data into dotStar format.  Convert to rgb888, apply brightness and gamma correction
    buildColumn(dst,colPtr);

    // Turn on the selected colunm bus buffer
    digitalWrite(ringEnable[ringIndex], LOW); // enable
    delayMicroseconds(1); // settle

    startColumnDma(dst);

    // Wait for the DMA to finish this column before sending the next one.
    // This is a simple approach; can be optimized to queue multiple transfers.
    while (dmaBusy) {
      pollDmaComplete();
    }
    digitalWrite(ringEnable[ringIndex], HIGH); // disaable
  }
}

//#################################################################################
// Allocate contiguous memory for the entire frame (COLUMNS * COLUMN_PAYLOAD)
//#################################################################################
void ensureBuffersAllocated() {
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
void freeBuffers() {
  if (frontBuffer) { free(frontBuffer); frontBuffer = nullptr; }
  if (backBuffer)  { free(backBuffer);  backBuffer  = nullptr; }
}

//########################################################
// Atomic swap of front/back pointers. Must be fast.
//########################################################
void swapBuffersAtomic() {
  noInterrupts();
  uint8_t *tmp = frontBuffer;
  frontBuffer = backBuffer;
  backBuffer = tmp;
  interrupts();
}

//############################################################
// Build one dotStar dotStar column (start+48*4+end) into dst
//############################################################
void buildColumn(uint8_t *dst, uint8_t *colPtr) {
  int idx = 0;

  // Start frame (32 bits of zero)
  dst[idx++] = 0x00;
  dst[idx++] = 0x00;
  dst[idx++] = 0x00;
  dst[idx++] = 0x00;

  // Build dotStar data for each LED in the column
  for (int i = 0; i < LEDS_PER_COLUMN; ++i) {

    // Extract the rgb fields for this LED row
    uint8_t r = colPtr[i*COLUMNS*3];
    uint8_t g = colPtr[i*COLUMNS*3+1];
    uint8_t b = colPtr[i*COLUMNS*3+2];

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
void initSpi() {
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
void startColumnDma(uint8_t *columnData) {
  if (!spi) return;

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
  dmaBusy = true;

  // Queue transaction (non-blocking)
  esp_err_t ret = spi_device_queue_trans(spi, t, portMAX_DELAY);
  if (ret != ESP_OK) {
    Serial.printf("spi_device_queue_trans error %d\n", ret);
    dmaBusy = false;
    free(t);
  }
}

//########################################################
// Poll for DMA completion and free transaction
//########################################################
void pollDmaComplete() {
  if (!dmaBusy) return;

  spi_transaction_t *rtrans;
  esp_err_t ret = spi_device_get_trans_result(spi, &rtrans, 0); // timeout 0 => non-blocking
  if (ret == ESP_OK && rtrans != nullptr) {

    // Completed transaction
    // Free the transaction memory we allocated in startColumnDma
    heap_caps_free(rtrans);
    dmaBusy = false;
  }
  // if ret == ESP_ERR_TIMEOUT -> not finished yet; do nothing
}

