// POV_DotStar_DMA.ino
// ESP32 Arduino sketch: APA102 (DotStar) driving with DMA, double-buffered,
// 4x-per-revolution, and CD4052 ring multiplexing.
//
// Hardware assumptions:
// - ESP32 (Arduino core)
// - APA102 / DotStar LED chains: 4 rings, each 48 LEDs (total framebuffer 120x48).
// - A SN74AHCT125 tri-state bus buffer to switch MOSI to one of 4 rings.
// - Break-beam / optical sensor -> digital input pin (generates 4 pulses/per rev).
//
// Pin examples (change to match your wiring):

#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <images.h>
#include <Adafruit_AS5600.h>

const int PIN_SPI_MOSI = 13;  // MOSI (APA102 DATA)
const int PIN_SPI_SCLK = 14;  // SCLK (APA102 CLK)
const int RING0_ENB = 2;       // SN74AHCT125 bus driver bit 0 select
const int RING1_ENB = 4;       // SN74AHCT125 bus driver bit 1 select
const int RING2_ENB = 16;      // SN74AHCT125 bus driver bit 2 select
const int RING3_ENB = 17;      // SN74AHCT125 bus driver bit 3 select

// LED / frame geometry
const int ROWS = 48;            // vertical rows (LEDs per ring)

const int TOTAL_COLUMNS = 120;  // total angular columns in a revolution
const int RINGS = 4;            // number of rings
const int COLS_PER_RING = TOTAL_COLUMNS/RINGS;            // number of rings
const int ringEnable[] = {RING0_ENB, RING1_ENB, RING2_ENB, RING3_ENB};

// There are 120 columns in 360 degrees of the sphere.  One for every 3 degrees.
int currentColumn;    // The one ranges between 0-120.         

// APA102 specifics
const int BYTES_PER_LED = 4;  // APA102 uses 4 bytes per LED (global, B, G, R in common libs)

// We'll build each column as: [4-byte start frame][48 * 4 bytes LED frames][4-byte end frame]
const int START_FRAME_BYTES = 4;
const int END_FRAME_BYTES = 4;
const int LEDS_PER_COLUMN = ROWS;
const int COLUMN_PAYLOAD = START_FRAME_BYTES + (LEDS_PER_COLUMN * BYTES_PER_LED) + END_FRAME_BYTES;
const int TOTAL_COLUMNS_BYTES = TOTAL_COLUMNS * COLUMN_PAYLOAD;
float lastAngleInDegrees = 0.0;
float curAngleInDegrees = 0.0;

//##########################
// PID motor speed control
//##########################
const int MOTOR_PWM_PIN = 25; // To control the motor speed

Adafruit_AS5600 as5600;  // magnetic angle sensor
const uint32_t samplePeriod_us = 10000; // 10 ms
uint16_t lastAngle = 0;
uint32_t lastTime  = 0;
float motorRPM = 0.0f;

const uint32_t MIN_DT_US = 10000; // ignore pulses closer than 10ms => noise filter
const uint32_t RPM_TIMEOUT_MS = 200; // if no pulses for 200ms, zero RPM

// --- shared state (ISR/main)
volatile uint32_t lastPulseTime = 0; // micros() at last valid pulse
volatile uint32_t pulseDt = 0;       // dt in microseconds between last two valid pulses
volatile uint32_t lastPulseMillis = 0; // millis() when last valid pulse arrived

// ====== PID control ======
float targetRPM = 420.0;

float Kp = 0.2f;
float Ki = 0.8f;
float Kd = 0.02f;
float Kff = 0.55f; // small feedforward (adjustable)

// Runtime state
float pidIntegral = 0.0f;
float lastError = 0.0f;
float lastDerivative = 0.0f; // filtered derivative
uint32_t lastPidMs = 0;  // Last time PID was updated

// Output limits (map to your PWM range)
const float PWM_MIN = 200.0f;
const float PWM_MAX = 255.0f;

const float DERIV_FILTER_TAU = 0.05f; // derivative low-pass (seconds). 0.01..0.2 typical

// SPI and DMA objects
spi_device_handle_t spi = nullptr;

// Double buffers allocated on heap (to make swapping trivial)
uint8_t *frontBuffer = nullptr; // displayed buffer (contains TOTAL_COLUMNS columns sequentially)
uint8_t *backBuffer  = nullptr; // written by CPU (next frame)
uint8_t backBufferFillIndex = 0;  // We only write the backbuffer a few columns at a time.  This index points to the next column to write.

// dlf  Remove this once I put in the motor PID/PWM code
unsigned long simulatedMotorSensorTimer = 0; 

volatile bool dmaBusy = false; // indicates a DMA transfer is in flight

// ---------- Forward declarations ----------
void setupMotor() ;
float updatePID(float rpmMeasured, float targetRPM);
void initSpi();
void buildColumn(uint8_t *dst, uint32_t rgb48[]);
void startColumnDma(uint8_t *columnData);
void pollDmaComplete();
void fillWholeBackbuffer(); 
void fillBackBufferPartially(uint8_t index);
void swapBuffersAtomic();
void ensureBuffersAllocated();
void freeBuffers();

//#########################################
// Setup Section
//#########################################
void setup() {
  Serial.begin(115200);
  delay(2000);

  // Pin setup
  pinMode(RING0_ENB, OUTPUT);
  pinMode(RING1_ENB, OUTPUT);
  pinMode(RING2_ENB, OUTPUT);
  pinMode(RING3_ENB, OUTPUT);

  setupMotor();

  // Uses i2c interface to AS5600 to read angle
  if (!as5600.begin()) {
    Serial.println("Could not find AS5600 sensor, check wiring!");
    while (1) {
      delay(10);
    }
  }

  // Allocate buffers
  ensureBuffersAllocated();

  // Pre-build initial frames (fill backBuffer with something)
  fillWholeBackbuffer();

  // Initialize SPI with DMA
  initSpi();

  // Small stabilization time
  delay(10);

  Serial.println("Setup complete.");
}

//#########################################
// Main Loop
//#########################################
void loop() {
  // Poll DMA completion regularly (non-blocking)
  pollDmaComplete();

  // Check the motor speed
  uint32_t now = micros();

  if (now - lastTime >= samplePeriod_us) {
    uint16_t curr = as5600.getRawAngle();
    int16_t delta = curr - lastAngle;

    // Takes care of case where we cross the 360 degree back to 0 degree boundary
    if (delta > 2048)  delta -= 4096;
    if (delta < -2048) delta += 4096;

    // Compute rpm by measuring the delta-angle between polling
    float dt = (now - lastTime) * 1e-6;
    motorRPM = (delta * 60.0) / (4096.0 * dt);

    lastAngle = curr;
    lastTime = now;
  }

  // ====== PID speed regulation ======
  if (millis() - lastPidMs > 100) {
    float pwm = updatePID(motorRPM, targetRPM);
    ledcWrite(0, (int)pwm);
    //Serial.printf("RPM: %.2f   PWM: %.2f\n", motorRPM, pwm);
  }

  // See what angle the sphere is at
  uint16_t curRawAngle = as5600.getRawAngle();
  curAngleInDegrees = ((curRawAngle/4096.0) * 360.0);
  float deltaAngle = curAngleInDegrees - lastAngleInDegrees;

  // Takes care of case where we cross the 360 degree back to 0 degree boundary
  if (deltaAngle > 180.0)  deltaAngle -= 360.0;
  if (deltaAngle < -180.0) deltaAngle += 360.0;

  // Every three-degree boundary we will start another column transfer.  This keeps the column displays perfectly aligned even if the motor speed varies
  if(deltaAngle >= 3.0) {
    lastAngleInDegrees = curAngleInDegrees;

    // Find out which framebuffer-relative column we are at
    currentColumn = int(((curRawAngle/4096.0) * 360.0) / 3.0);
    //Serial.printf("rawAngle: %d  curAngle: %.2f   curColumn: %d   deltaAngle: %.2f\n", curRawAngle, curAngleInDegrees,currentColumn, deltaAngle);

    // No display update if we are still transferring data from the last update...
    if (!dmaBusy) {

      // Point the frontbuffer to the new data
      //dlf swapBuffersAtomic();

      // Need to reverse the column index since the sphere is rotating clockwise which means it's painting right to left from the framebuffer (i.e. highest index to lowest)
      int reversedColumn = TOTAL_COLUMNS - 1 - currentColumn;

      // for each column, stream out the four rings led data
      for(int ringIndex = 0; ringIndex < 4; ringIndex++) {
        uint8_t baseCol = ringIndex * COLS_PER_RING;
        uint8_t *colPtr = frontBuffer + (((baseCol + reversedColumn) % TOTAL_COLUMNS) * COLUMN_PAYLOAD);  // modulo 120 so we wrap when not starting at col-0 
        //uint8_t *colPtr = frontBuffer + (((baseCol + c) % TOTAL_COLUMNS) * COLUMN_PAYLOAD);  // modulo 120 so we wrap when not starting at col-0 
        //Serial.printf("curCol= %d   reversedCol=%d  ri=%d   colPtr=%d\n",currentColumn,reversedColumn,ringIndex,(baseCol+c)%TOTAL_COLUMNS);

        // Turn on the selected colunm bus buffer
        digitalWrite(ringEnable[ringIndex], LOW); // enable
        delayMicroseconds(1); // settle

        startColumnDma(colPtr);

        // Wait for the DMA to finish this column before sending the next one.
        // This is a simple approach; can be optimized to queue multiple transfers.
        while (dmaBusy) {
          pollDmaComplete();
         
          // While waiting for the DMA to finish, do a partial update of the backbuffer.  There isn't enough time to fill the
          // entire buffer at once, so we do a little bit during each column window.
          fillBackBufferPartially(backBufferFillIndex);
          if(backBufferFillIndex == TOTAL_COLUMNS - 4) {
            backBufferFillIndex = 0;
          }
        }
        digitalWrite(ringEnable[ringIndex], HIGH); // disaable
      }
    }
  }
}

// #############################################################
//  Functions
// #############################################################

// Allocate contiguous memory for the entire frame (TOTAL_COLUMNS * COLUMN_PAYLOAD)
void ensureBuffersAllocated() {
  if (frontBuffer == nullptr) {
    frontBuffer = (uint8_t*)heap_caps_malloc(TOTAL_COLUMNS_BYTES, MALLOC_CAP_8BIT);
  }
  if (backBuffer == nullptr) {
    backBuffer  = (uint8_t*)heap_caps_malloc(TOTAL_COLUMNS_BYTES, MALLOC_CAP_8BIT);
  }
  if (!frontBuffer || !backBuffer) {
    Serial.println("ERROR: buffer allocation failed. Reduce buffer sizes or check memory.");
    while (1) { delay(1000); }
  }
}

void freeBuffers() {
  if (frontBuffer) { free(frontBuffer); frontBuffer = nullptr; }
  if (backBuffer)  { free(backBuffer);  backBuffer  = nullptr; }
}

void swapBuffersAtomic() {
  // Atomic swap of front/back pointers. Must be fast.
  noInterrupts();
  uint8_t *tmp = frontBuffer;
  frontBuffer = backBuffer;
  backBuffer = tmp;
  interrupts();
}

// Fill the backbuffer four columns at a time since we don't have time to fill it completely between updating led strips
void fillBackBufferPartially(uint8_t index){
  for (int col = index; col < index+4; ++col) {

    // Build an array of 48 RGB values (packed 0xRRGGBB)
    uint32_t rgb48[LEDS_PER_COLUMN];
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    for (int row = 0; row < LEDS_PER_COLUMN; ++row) {

      if(image1[row][col] == 1) {
        red =  128;
      } else {
        red =  0;
      }
      green = 0;
      blue = 0;
      rgb48[row] = (red << 16) | (green << 8) | (blue);
    }

    // Build the APA102-formatted column into backBuffer
    uint8_t *dst = backBuffer + (col * COLUMN_PAYLOAD);
    buildColumn(dst, rgb48);
  }
}

// Read and fill the backbuffer
void fillWholeBackbuffer() {

  for (int col = 0; col < TOTAL_COLUMNS; ++col) {

    // Build an array of 48 RGB values (packed 0xRRGGBB)
    uint32_t rgb48[LEDS_PER_COLUMN];
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    for (int row = 0; row < LEDS_PER_COLUMN; ++row) {

      if(image1[row][col] == 1) {
        red =  128;
      } else {
        red =  0;
      }
      green = 0;
      blue = 0;
      rgb48[row] = (red << 16) | (green << 8) | (blue);
    }

    // Build the APA102-formatted column into backBuffer
    uint8_t *dst = backBuffer + (col * COLUMN_PAYLOAD);
    buildColumn(dst, rgb48);

    //dlf  just as a test, fill the front buffer and comment out the swap in the main loop (i.e. just use frontbuffer)
    uint8_t *dst1 = frontBuffer + (col * COLUMN_PAYLOAD);
    buildColumn(dst1, rgb48);
  }
}

// Build one APA102 column (start+48*4+end) into dst
void buildColumn(uint8_t *dst, uint32_t rgb48[]) {
  int idx = 0;

  // start frame (4 bytes zero)
  dst[idx++] = 0x00;
  dst[idx++] = 0x00;
  dst[idx++] = 0x00;
  dst[idx++] = 0x00;

  // 48 LED frames: [global brightness][B][G][R] (we'll set global brightness = 0xE0 | 31 for max)
  for (int i = 0; i < LEDS_PER_COLUMN; ++i) {
    uint8_t r = (rgb48[i] >> 16) & 0xFF;
    uint8_t g = (rgb48[i] >> 8) & 0xFF;
    uint8_t b = (rgb48[i]) & 0xFF;
    //dlf dst[idx++] = 0xE0 | 0x1F; // global brightness max (0xE0 + 5-bit)
    dst[idx++] = 0xE0 | 0x07; // global low brightness  (0xE0 + 5-bit)
    dst[idx++] = b;
    dst[idx++] = g;
    dst[idx++] = r;
  }

  // end frame -- a small number of 0xFF bytes recommended; using 4 here
  dst[idx++] = 0xFF;
  dst[idx++] = 0xFF;
  dst[idx++] = 0xFF;
  dst[idx++] = 0xFF;
}

// ---------- SPI DMA setup ----------
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

  // device config (APA102 doesn't use CS)
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

// Start a DMA transfer of exactly one column (non-blocking).
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

// Poll for DMA completion and free transaction
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



// Update the PWM value sent to the motor via a PID loop
// rpmMeasured = measured RPM (float),  targetRPM = desired RPM (global)
float updatePID(float rpmMeasured, float targetRPM) {
  uint32_t now = millis();
  float dt = (lastPidMs == 0) ? 0.05f : ( (now - lastPidMs) / 1000.0f );
  lastPidMs = now;
  if (dt <= 0.0f) dt = 0.05f; // safety fallback

  // Error
  float error = targetRPM - rpmMeasured;

  // Integral term (with clamping to avoid wind-up)
  pidIntegral += error * dt;

  // compute I term candidate and clamp integral so I stays within output limits
  float Iterm = Ki * pidIntegral;

  // We'll clamp integral using a conservative bound to keep Iterm from driving output outside limits.
  // Compute allowable Iterm bounds:
  float Pterm = Kp * error;

  // approximate derivative (unfiltered) for bounds calculation:
  float derivUnfiltered = (error - lastError) / dt;
  float Dterm = Kd * lastDerivative; // use filtered derivative for display (we'll compute below)

  // Compute available headroom for Iterm
  float maxI = PWM_MAX - (Pterm + Kff * targetRPM); // conservative
  float minI = PWM_MIN - (Pterm + Kff * targetRPM);

  // Clamp Iterm then clamp pidIntegral accordingly
  if (Iterm > maxI) {
    Iterm = maxI;
    pidIntegral = Iterm / Ki;
  } else if (Iterm < minI) {
    Iterm = minI;
    pidIntegral = Iterm / Ki;
  }

  // Derivative (per second), then low-pass filter it
  float derivative = (error - lastError) / dt;

  // low-pass filter: alpha = dt / (tau + dt)
  float alpha = dt / (DERIV_FILTER_TAU + dt);
  lastDerivative = lastDerivative + alpha * (derivative - lastDerivative);
  Dterm = Kd * lastDerivative;

  // Feedforward (simple linear)
  float FFterm = Kff * targetRPM;

  // Combine
  float output = Pterm + Iterm + Dterm + FFterm;

  // clamp final output
  if (output > PWM_MAX) output = PWM_MAX;
  if (output < PWM_MIN) output = PWM_MIN;

  // Save error for next step
  lastError = error;

  // Debug print
  //Serial.printf("P:%.3f I:%.3f D:%.3f FF:%.3f -> out:%.1f (dt=%.3f)\n",
  //              Pterm, Iterm, Dterm, FFterm, output, dt);

  return output;
}

// Assign pins and be sure motor is off
void setupMotor() {
    ledcSetup(0, 20000, 8);   // 20 kHz PWM, 8-bit resolution
    ledcAttachPin(MOTOR_PWM_PIN, 0);

    // Start the motor spinning
    //ledcWrite(0, 128);
    ledcWrite(0, 0);
}

