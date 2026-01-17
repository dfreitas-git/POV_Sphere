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
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <Adafruit_AS5600.h>
#include <gimp_compat.h>
#include <images.h>
#include <gamma.h>
#include <ClickEncoder.h>
#include <Ticker.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>  // if we ever use GPIO.out_w1ts

int printCount = 0;  //Used to print limited debug messages in the loop so we don't flood the serial monitor

/* ===================== Menu Types ===================== */
enum MenuItemType {
  MENU_SUBMENU,
  MENU_ACTION,
  MENU_INT_VALUE,
  MENU_FLOAT_VALUE,
  MENU_LIST
};

// Prototypes
struct MenuItem;
typedef void (*ActionCallback)(MenuItem*);
void motorOnOff(MenuItem*) ;
void scrollOnOff(MenuItem*) ;
void encoderService() ;
void buildMenu() ;
void drawMenu() ;
void handleRotation(int delta) ;
void handleClick() ;
void handleDoubleClick() ;
void updateBlink() ;
void setupMotor() ;
void initSpi();
void buildColumn(uint8_t *dst, uint8_t *colPtr);
void startColumnDma(uint8_t *columnData);
void pollDmaComplete();
void updateColumnLEDs(uint16_t columnIndex);
void swapBuffersAtomic();
void ensureBuffersAllocated();
void freeBuffers();
void uiTask(void* parameter) ;
float updatePID(float rpmMeasured, float targetRPM);
void computeCurrentAngle();

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

// ###########################################################
//   DotStar, LED, DMA, critical timing code running on core-1
// ###########################################################
// DotStar /  byte order
// NOTE: Many "APA102-compatible" strips (e.g. SK9822) may use GRB internally.
#define DOTSTAR_ORDER_GRB   0   // set to 0 for true APA102 (BGR)
#define GIMP_RGB565_LITTLE_ENDIAN  1  // Gimp ouputs rgb565 in little endian byte ordering
#define BRIGHTNESS_R 200  // 0–255 (≈75%)  Use this to brighten/dim LED's after gamma correction
#define BRIGHTNESS_G 225  // 0–255 (≈75%)  Use this to brighten/dim LED's after gamma correction
#define BRIGHTNESS_B 255  // 0–255 (≈75%)  Use this to brighten/dim LED's after gamma correction

const int PIN_SPI_MOSI = 13;  // MOSI ( DATA)
const int PIN_SPI_SCLK = 14;  // SCLK ( CLK)
const int RING0_ENB = 2;       // SN74AHCT125 bus driver bit 0 select
const int RING1_ENB = 4;       // SN74AHCT125 bus driver bit 1 select
const int RING2_ENB = 16;      // SN74AHCT125 bus driver bit 2 select
const int RING3_ENB = 17;      // SN74AHCT125 bus driver bit 3 select

// LED / frame geometry
constexpr uint32_t AS5600_COUNTS = 4096;  // number of counts in 360 degrees from the AS5600 sensor
const int ROWS = 48;                      // vertical rows (LEDs per ring)
constexpr uint32_t COLUMNS = 120;         // total angular columns in a revolution
const int RINGS = 4;                      // number of LED rings
const int COLS_PER_RING = COLUMNS/RINGS;  // number of columns each ring fills
const int ringEnable[] = {RING0_ENB, RING1_ENB, RING2_ENB, RING3_ENB};  // Pins that control the mux for the serial DMA

// dotStar specifics
const int BYTES_PER_LED = 4;       // dotStart uses 4 bytes per LED (global, B, G, R in common libs)
const int START_FRAME_BYTES = 4;
const int END_FRAME_BYTES = 4;
const int LEDS_PER_COLUMN = ROWS;
const int FRAME_COLUMN_BYTES = LEDS_PER_COLUMN * 3; // one byte per R/G/B
const int TOTAL_COLUMNS_RGB_BYTES = COLUMNS * LEDS_PER_COLUMN * 3;  // total number of bytes in the framebuffers

// We'll build each column as: [4-byte start frame][48 * 4 bytes LED frames][4-byte end frame]
const int COLUMN_PAYLOAD = START_FRAME_BYTES + (LEDS_PER_COLUMN * BYTES_PER_LED) + END_FRAME_BYTES;
const int TOTAL_COLUMNS_BYTES = COLUMNS * COLUMN_PAYLOAD;

#define SCROLL_UPDATE_TIME 50      // How often (in milliseconds) to update the framebuffer offset pointer.  Controls how fast the image scrolls around the Sphere.
uint16_t framebufferOffset=0;      // Shift where in the frame buffer we get the column to display.  Use this to scroll the image.
unsigned long lastScrollTime;
unsigned long lastAnimateTime;     

// Core-0 will do the actual angle measurements, Core-1 will sync to them
volatile uint32_t measuredAngle;   // AS5600 raw
uint32_t lastMeasuredTime;         // micros() timestamp
#define MAX_TRIM 2                 // The most we allow core-0 to adjust the angle being computed by core-1

// Core-1 angle values calculated and pll-locked to core-0 actual angle measurements
#define OMEGA_SHIFT 16
#define OMEGA_TRIM_PERIOD 30000000  // The amount of time (in microseconds) we'd want to apply frequency trim over to our core-1 VCO
#define PHASE_DEADBAND 10           // Any angle error less than this and we don't apply any more correction to the VCO

// Core-1 PLL variables
int64_t angle_accum;            // 64-bits so integer math doesn't lose remainder precision
int32_t angle_q;                // current predicted angle (Q0, 0–4095)
volatile int32_t omega_ff;      // angle counts per microsecond (Scaled by OMEGA_SHIFT for integer math)
int32_t core_1_omega_ff;        // local copy used by core-1.  Core-1 will add any necessary phase correction to it.
uint16_t core_1_omega_trim;     // local copy used by core-1.
uint32_t lastAngleTime;         // Used by core-1 to calculate dt between angle calculations
long phase_error;               // Current error between measured angle from core-0 and computed angle on core-1
volatile uint16_t omega_trim;   // Accumulated error between measured angle from core-0 and computed angle on core-1

// Core-1 column position vars
uint32_t nextColumnAngle = 0;
uint16_t columnIndex = 0;

// Use integer math; keep remainder for precision
constexpr uint32_t COLUMN_STEP = AS5600_COUNTS / COLUMNS;      // 34
constexpr uint32_t COLUMN_REM  = AS5600_COUNTS % COLUMNS;      // 16

//##########################
// PID motor speed control
//##########################
const int MOTOR_PWM_PIN = 25; // To control the motor speed

Adafruit_AS5600 as5600;  // magnetic angle sensor
const uint32_t samplePeriod_us = 20000; // minimum time (us) between sampling the angle measurement in core-0
#define PID_UPDATE_TIME 100      // How many milliseconds between updating the motor PWM.
uint16_t lastAngle = 0;
float motorRPM = 0.0f;

// ====== PID control ======
float targetRPM = 360.0;

// Motor control PID parameters
float Kp = 0.2f;
float Ki = 0.8f;
float Kd = 0.02f;
float Kff = 0.55f; // small feedforward (adjustable)

// Runtime state
float pidIntegral = 0.0f;
float lastError = 0.0f;
float lastDerivative = 0.0f; // filtered derivative
uint32_t lastPidMs = 0;  // Last time PID was updated

// Output limits (map to motor PWM range)
const float PWM_MIN = 125.0f;
const float PWM_MAX = 255.0f;

const float DERIV_FILTER_TAU = 0.05f; // derivative low-pass (seconds). 0.01..0.2 typical

// SPI and DMA objects
spi_device_handle_t spi = nullptr;

// Double buffers allocated on heap (to make swapping trivial)
uint8_t *frontBuffer = nullptr;   // Used by core-1.  Displayed buffer (contains COLUMNS columns sequentially)
uint8_t *backBuffer  = nullptr;   // Written by core-0 (next frame)
volatile bool backBufferFilled = false;  // Set to true when a valid image has been loaded into it.

volatile bool dmaBusy = false; // indicates a DMA transfer is in flight

// ###########################################################
//   UI core-0 OLED/rotary-encoder/switch definitions
// ###########################################################
/* ===================== OLED ===================== */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SDA_OLED 32
#define SCL_OLED 33
#define OLED_RESET    -1     // no reset pin
#define OLED_ADDR     0x3C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

/* ===================== Encoder ===================== */
#define ENC_A 35
#define ENC_B 34
#define ENC_BTN 27
ClickEncoder encoder(ENC_A, ENC_B, ENC_BTN, 4);
Ticker encoderTicker;

/* ===================== Menu Item ===================== */
struct MenuItem {
  const char* name;
  MenuItemType type;
  MenuItem* parent;

  /* For submenu */
  MenuItem** children;
  uint8_t childCount;

  /* For callbacks */
  ActionCallback callback;

  /* For int value */
  int* intValue;
  int minIntVal;
  int maxIntVal;

  /* For float value */
  float* floatValue;
  float minFloatVal;
  float maxFloatVal;

  /* For option list */
  const char** options;
  uint8_t optionCount;
  uint8_t* optionIndex;
};

TaskHandle_t uiTaskHandle;

/* ===================== Global Menu State ===================== */
MenuItem* currentMenu;
uint8_t currentIndex = 0;
bool editingValue = false;

/* Blink control */
bool blinkOn = true;
uint32_t lastBlink = 0;
const uint32_t blinkInterval = 400;  // ms


/* ===================== Encoder ISR ===================== */
void encoderService() {
  encoder.service();
}

/* ===================== Global Variables ===================== */
int brightness = 3;  // Sphere LED brightness
uint8_t fiveBitBright; // hold the mapping of the menu brightness (0-10) to the dotStar five-bit brightness (0-31)
volatile bool motorOnOffFlag = 0;   // Turn the mhtor on/off
volatile bool scrollOnOffFlag = 0;   // Turn the scrolling of the image on/off
uint8_t imageToDisplayIndex = 0;

// Images to display on the Sphere
const int NUMBER_OF_DISPLAY_FILES = IMG_COUNT;
const char* imageToDisplay[IMG_COUNT];


/* ===================== Callbacks ===================== */

void motorOnOff(MenuItem*) {
  if(motorOnOffFlag == 1) {
    motorOnOffFlag = 0;
     Serial.println("Motor Off");
  } else {
    motorOnOffFlag = 1;
     Serial.println("Motor On");
  }
}

void scrollOnOff(MenuItem*) {
  if(scrollOnOffFlag == 1) {
    scrollOnOffFlag = 0;
     Serial.println("Scroll Off");
  } else {
    scrollOnOffFlag = 1;
     Serial.println("Scroll On");
  }
}

/* ===================== Menu Declarations ===================== */
MenuItem menuMain;
MenuItem menuSettings;
MenuItem menuBrightness;
MenuItem menuDisplay;
MenuItem menuRPM;
MenuItem menuMotorOnOff;
MenuItem menuScrollOnOff;

/* ===================== Menu Construction ===================== */
MenuItem* settingsChildren[] = {
  &menuBrightness,
  &menuRPM
};

MenuItem* mainChildren[] = {
  &menuMotorOnOff,
  &menuScrollOnOff,
  &menuDisplay,
  &menuSettings
};

void buildMenu() {

  menuMain = {
    "Main Menu",
    MENU_SUBMENU,
    nullptr,
    mainChildren,
    4,
    nullptr,
    nullptr, 0, 0,
    nullptr, 0, 0,
    nullptr, 0, nullptr
  };

  menuSettings = {
    "Settings",
    MENU_SUBMENU,
    &menuMain,
    settingsChildren,
    2,
    nullptr,
    nullptr, 0, 0,
    nullptr, 0, 0,
    nullptr, 0, nullptr
  };

  menuBrightness = {
    "Brightness",
    MENU_INT_VALUE,
    &menuSettings,
    nullptr, 0,
    nullptr,
    &brightness, 0, 10,
    nullptr, 0, 0,
    nullptr, 0, nullptr
  };

  for (int i=0;i<IMG_COUNT;i++) {
    imageToDisplay[i] = imageTable[i]->name;
  }

  menuDisplay = {
    "Display",
    MENU_LIST,
    &menuSettings,
    nullptr, 0,
    nullptr,
    nullptr, 0, 0,
    nullptr, 0, 0,
    imageToDisplay, NUMBER_OF_DISPLAY_FILES, &imageToDisplayIndex
  };

  menuMotorOnOff = {
    "Motor On/Off",
    MENU_ACTION,
    &menuSettings,
    nullptr, 0,
    motorOnOff,
    nullptr, 0, 0,
    nullptr, 0, 0,
    nullptr,0,nullptr
  };

  menuScrollOnOff = {
    "Scroll On/Off",
    MENU_ACTION,
    &menuSettings,
    nullptr, 0,
    scrollOnOff,
    nullptr, 0, 0,
    nullptr, 0, 0,
    nullptr,0,nullptr
  };

  menuRPM = {
    "Motor RPM",
    MENU_FLOAT_VALUE,
    &menuSettings,
    nullptr, 0,
    nullptr,
    nullptr, 0, 0,
    &targetRPM, 200.0, 400.0,
    nullptr, 0, nullptr
  };

  currentMenu = &menuMain;
}

/* ===================== Display ===================== */
void drawMenu() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  for (uint8_t i = 0; i < currentMenu->childCount; i++) {
    int y = i * 10;

    if (i == currentIndex) {
      oled.setCursor(0, y);
      oled.print(">");
    }
    oled.setCursor(10, y);
    oled.print(currentMenu->children[i]->name);

    MenuItem* item = currentMenu->children[i];
    bool showValue = true;
    if (editingValue && i == currentIndex) {
      showValue = blinkOn;
    }

    if (item->type == MENU_INT_VALUE && showValue) {
      oled.setCursor(90, y);
      oled.print(*item->intValue);
    }

    if (item->type == MENU_FLOAT_VALUE && showValue) {
      oled.setCursor(90, y);
      oled.print(int(*item->floatValue));
    }

    if (item->type == MENU_LIST && showValue) {
      oled.setCursor(60, y);
      oled.print(item->options[*item->optionIndex]);
    }
  }

  oled.display();
}

/* ===================== Input Handling ===================== */
void handleRotation(int delta) {
  MenuItem* item = currentMenu->children[currentIndex];

  if (editingValue) {
    if (item->type == MENU_INT_VALUE) {
      *item->intValue = constrain(
        *item->intValue + delta,
        item->minIntVal,
        item->maxIntVal
      );
    }
    if (item->type == MENU_FLOAT_VALUE) {
      *item->floatValue = constrain(
        *item->floatValue + delta,
        item->minFloatVal,
        item->maxFloatVal
      );
    }

    if (item->type == MENU_LIST) {

      // Wrap the point back to the start if we roll off the end of the list
      int idx = (*item->optionIndex + delta) % item->optionCount;
      if (idx < 0) {
        idx = idx + item->optionCount;
      }
      *item->optionIndex = idx;
    }
  } else {
    int newIndex = (currentIndex + delta) % currentMenu->childCount;
    if (newIndex < 0) {
      newIndex = newIndex + currentMenu->childCount;
    }
    currentIndex = newIndex;
  }
}

void handleClick() {
  MenuItem* item = currentMenu->children[currentIndex];

  if (item->type == MENU_SUBMENU) {
    currentMenu = item;
    currentIndex = 0;
  } else if (item->type == MENU_ACTION) {
    if (item->callback) {
      item->callback(item);
    }
  } else {
    editingValue = !editingValue;
  }
}

void handleDoubleClick() {
  if (editingValue) {
    editingValue = false;
    return;
  }

  if (currentMenu->parent) {
    currentMenu = currentMenu->parent;
    currentIndex = 0;
  }
}

// For blinking the edited menu item
void updateBlink() {
  uint32_t now = millis();
  if (now - lastBlink >= blinkInterval) {
    blinkOn = !blinkOn;
    lastBlink = now;
  }
}

//#########################################
// Main Setup Code
//#########################################
void setup() {
  Serial.begin(115200);
  delay(2000);

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

//##############################################
// Function to display a fade on the Sphere
//##############################################

//####################################
//  Slow fade in/out of solid colors
//
//  cycle increments once per full fade
//  phase (in radians) is always 0 … 2π within that cycle
//  Randomly switch colors at cycle boundaries
//  Brightness is just cos(phase) (shifted to 0-255 result)
//####################################
void fillBB_fade() {

  const float periodMs = 3000.0f;  // one full fade cycle

  // graphics are based on the actual time so frames are synced to the clock
  float t = millis();

  // What cycle are we in?
  uint32_t cycle = uint32_t(t / periodMs);
  static uint32_t lastCycle = 0;

  // Compute the phase angle (in radians 0-2pi) for the given time within the cycle
  float phase = (t - cycle * periodMs) * (2.0f * PI / periodMs);  

  // Brightness: [-1,1] -> [0,255]
  // Use cos so brightness peaks are in phase 
  //uint8_t bright = (1 - cos(phase)) * 0.5 * 255;
  float bright = (1 - cos(phase)) * 0.5;

  // Choose color deterministically
  //uint8_t r8 = 0, g8 = 0, b8 = 0;
  static uint8_t r8, g8, b8;
  static uint8_t r8Base, g8Base, b8Base;

  // Switches to the next color when the cycle count increments which is where cos(phase) is zero so we softly transition
  if(cycle != lastCycle) {
    lastCycle = cycle;
    r8Base = uint8_t(random(256));
    g8Base = uint8_t(random(256));
    b8Base = uint8_t(random(256));
  }
  r8 = r8Base * bright;
  g8 = g8Base * bright;
  b8 = b8Base * bright;

  for (uint8_t col = 0; col < COLUMNS; col++) {
    for (uint8_t row = 0; row < ROWS; row++) {
      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = r8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
    }
  }
}


//###############################################################
//  Color flowing out the top, down the Sphere in dripping sheets
//###############################################################
void fillBB_Flow() {

  // Create a head pointer which is the leading edge of the first spill color
  uint16_t spillTime_mS = 4000;   // Time to iterate acroll 48 rows
  uint32_t elapsed = millis();
  uint32_t head = (elapsed * 48) / spillTime_mS;   // What row the spill edge is at
  uint32_t cycle = elapsed / spillTime_mS;        // integer cycle count  
  head %= 48;

  // Struct to hold the rgb color fields
  struct RGB { uint8_t r, g, b; };

  // Pattern of how we want the boundary edge to look (just "paint" one half and we will tile it to both sides of the Sphere
  static const int8_t waveLUT[COLUMNS/2] = {
    -1,-1,0,0,-1,0,5,6,5,-3,-2,0,0,-1,-1,0,1,0,5,6,5,-3,-4,-3,0,0,-3,-4,-3,0,  1,0,0,0,1,10,11,10,0,-1,-2,-3,-2,-1,0,0,-1,0,5,8,5,1,0,-3,-3,0,-2,-1,-1,0,
  };
  
  // Set up some pre-determined colors to cycle between (these give good contrast)
  RGB colors[] = {
                  {255,0,0},
                  {0,255,0},
                  {1,65,223},
                  {10,229,0},
                  {218,24,94},
                  {74,160,240},
                  {231,121,67},
                  {72,106,16},
                  {101,16,187},
                  {5,17,137},
                  {196,16,127},
                 };
  static uint8_t bgColorIndex = 0;
  static uint8_t fgColorIndex = 1;
  uint8_t colorArrLen = sizeof(colors) / sizeof(colors[0]);

  // Create instances for the foreground and background color
  static RGB fgColor, bgColor;
  static uint32_t lastCycle = 0;
  uint8_t colorR, colorG, colorB;

  // head just wrapped from 47 → 0, swap rgb fg/bg colors and compute a new fg color
  if (cycle != lastCycle) {    
    bgColor = colors[bgColorIndex];
    fgColor = colors[fgColorIndex];
    bgColorIndex = fgColorIndex;
    fgColorIndex++;
    if(bgColorIndex == colorArrLen - 1) {
       bgColorIndex = 0;
    }
    if(fgColorIndex == colorArrLen - 1 ) {
       fgColorIndex = 0;
    }
    lastCycle = cycle;
  }

  // for each row, figure out which color band it lays in
  for (int row = 0; row < 48; row++) {
    for (int col = 0; col < 120; col++) {

      // Shape the boundary with offsets from our look up table
      int curHead = head + waveLUT[col % COLUMNS/2];
      RGB color = (row <= curHead) ? fgColor : bgColor;

      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = color.r;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = color.g;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = color.b;
    }
  }
}

//#############################################
//  Horizontal bands moving down the Sphere 
//#############################################
void fillBB_hBands() {

  // Create a head pointer which is the leading edge of the first spill color
  uint32_t elapsed = millis();
  uint8_t head = (elapsed * 48) / 3000;      // 0–47 over 3 seconds
  head %= 48;  // Wrap back the the start once head reaches the bottom of the Sphere

  // load with R, G, B for index 0,1,2
  const uint8_t r[3] = {255,0,0};
  const uint8_t g[3] = {0,255,0};
  const uint8_t b[3] = {0,0,255};

  // for each row, figure out which color band it lays in
  for (int row = 0; row < 48; row++) {
    int d = (head - row + 48) % 48;   // distance behind the head this current row is
    uint8_t colorIndex;
    if (d < 8)        colorIndex=0;
    else if (d < 16)   colorIndex=1;
    else if (d < 24)   colorIndex=2;
    else if (d < 32)   colorIndex=0;
    else if (d < 40)   colorIndex=1;
    else if (d < 48)   colorIndex=2;

    // Now write that color all the way around the Sphere
    for (int col = 0; col < 120; col++) {
        uint8_t r8 = 0, g8 = 0, b8 = 0;
        backBuffer[(row * COLUMNS * 3) + (col * 3)]     = r[colorIndex];
        backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g[colorIndex];
        backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b[colorIndex];
    }
  }
}

//#############################################
//  Horizontal stripes with fading color changes
//#############################################
void fillBB_hFade() {

  // Color fade speed
  const float periodMs = 3000.0f;  // one full fade cycle

  // animation speed.  1.0 Hz = one cycle every second
  float omega = 2.0f * PI * 2.0f;     
  //float omega = 2.0f * PI * 0.5f;     

  // graphics are based on the actual time so frames are synced to the clock
  float t = millis();
  float tSec = t * 0.001f;   // seconds
  float animatePhase = omega * tSec;  // Convert to phase to move the image

  // What cycle are we in?
  uint32_t cycle = uint32_t(t / periodMs);

  // Compute the phase angle (in radians) for the given time within the cycle
  float phase = (t - cycle * periodMs) * (2.0f * PI / periodMs);  

  // Brightness: [-1,1] -> [0,255]
  // Use cos so brightness peaks are in phase 
  uint8_t colorBright = (1 - cos(phase)) * 0.5 * 255;

  // Precompute sin of phi to keep it out of the inner loop
  float rowSin[ROWS];
  for (int row = 0; row < ROWS; row++) {
    float phi = ((float(row) / float(ROWS)) * PI) - PI/2.0;
    rowSin[row] = sin(phi * 8 + animatePhase);
  }
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      uint8_t bright = uint8_t((rowSin[row] * 0.5f + 0.5f) * colorBright);
      uint8_t r8 = 0, g8 = 0, b8 = 0;
      switch (cycle % 3) {
        case 0: r8 = bright; break;
        case 1: g8 = bright; break;
        case 2: b8 = bright; break;
      }
      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = r8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
    }
  }
}

//#############################################
//  Vertical stripes with fading color changes
//#############################################
void fillBB_vFade() {

  // Color fade speed
  const float periodMs = 3000.0f;  // one full fade cycle

  // animation speed.  1.0 Hz = one cycle every second
  float omega = 2.0f * PI * 5.0f;     

  // graphics are based on the actual time so frames are synced to the clock
  float t = millis();
  float tSec = t * 0.001f;   // seconds
  float animatePhase = omega * tSec;  // Convert to phase to move the image

  // What cycle are we in?
  uint32_t cycle = uint32_t(t / periodMs);

  // Compute the phase angle (in radians) for the given time within the cycle
  float phase = (t - cycle * periodMs) * (2.0f * PI / periodMs);  

  // Brightness: [-1,1] -> [0,255]
  // Use cos so brightness peaks are in phase 
  uint8_t colorBright = (1 - cos(phase)) * 0.5 * 255;

  // Now map the brightness based on the position
  for (int col = 0; col < COLUMNS; col++) {
    float theta = (col / float(COLUMNS)) * 2*PI;
    for (int row = 0; row < ROWS; row++) {
      float phi = (row / float(ROWS)) * PI - PI/2;

      uint8_t bright = uint8_t((sin(theta * 12 + animatePhase) * 0.5f + 0.5f) * colorBright);

      uint8_t r8 = 0, g8 = 0, b8 = 0;
      switch (cycle % 3) {
        case 0: r8 = bright; break;
        case 1: g8 = bright; break;
        case 2: b8 = bright; break;
      }
      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = r8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
    }
  }
}

//###################################################################
// Fill the backbuffer in preperation for the next displayed frame
// Assumes a 120x48 image imported from Gimp in rgb565 format (two
// bytes per pixel).  Images stored in include/images.h
//###################################################################
void fillBB_image() {

  // Reading an image created by Gimp (loaded from images.h)
  // and expanding/writing the bytes into the framebuffer.  framebuffer always
  // is loaded with rgb888  (one byte per r, g, b) per pixel.
  for (unsigned col = 0; col < imageTable[imageToDisplayIndex]->width; col++) {

    // Start at row 0, this column in the Gimp pixel array
    const uint8_t* p = imageTable[imageToDisplayIndex]->pixel_data + (col * 2);
    for (unsigned row = 0; row < imageTable[imageToDisplayIndex]->height; row++) {
      #if GIMP_RGB565_LITTLE_ENDIAN
        uint16_t rgb565 = (p[1] << 8) | p[0];
      #else
        uint16_t rgb565 = (p[0] << 8) | p[1];
      #endif

      // Read each of the sub-fields to extract rgb from the Gimp 2-byte data
      uint8_t r5 = (rgb565 >> 11) & 0x1F;
      uint8_t g6 = (rgb565 >> 5)  & 0x3F;
      uint8_t b5 =  rgb565        & 0x1F;

      // Expand 5/6-bit channels to full 8-bit range (0–255)
      uint8_t r8 = (r5 << 3) | (r5 >> 2);
      uint8_t g8 = (g6 << 2) | (g6 >> 4);
      uint8_t b8 = (b5 << 3) | (b5 >> 2);

      // Now store the rgb (three bytes per row) data into the backbuffer.  In the rendering (by core-1), 
      // we will add the start bytes, brightness, gamma, and stop bytes before DMA to the dotStars.
      backBuffer[(row * COLUMNS * 3) + (col * 3)] = r8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;

      // Advance to next row, same column
      p += imageTable[imageToDisplayIndex]->width * 2;
    }
  }
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

//########################################################
// ---------- SPI DMA setup ----------
//########################################################
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

//########################################################
// Start a DMA transfer of exactly one column (non-blocking).
//########################################################
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

//########################################################
// Update the PWM value sent to the motor via a PID loop
// rpmMeasured = measured RPM (float),  targetRPM = desired RPM (global)
//########################################################
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

//#######################################
// Assign pins and be sure motor is off
//#######################################
void setupMotor() {
    ledcSetup(0, 20000, 8);   // 20 kHz PWM, 8-bit resolution
    ledcAttachPin(MOTOR_PWM_PIN, 0);
    ledcWrite(0,0);
}

//########################################################
// Core-1 computes the angle the Sphere every loop cycle.   
// We do actual measurements in core-0 at a more relaxed 
// rate and just sync-up core-1 periodically.  That allows 
// us to remove the very slow magnetic  angle sensor 
// (AS5600) from the fast LED/DMA/render loop and replace 
// it with fast calculations so the max rpm can be increased.
//########################################################
void computeCurrentAngle() {
    uint32_t now = micros();
    uint32_t dt = now - lastAngleTime;
    lastAngleTime = now;

    angle_accum += (core_1_omega_ff + core_1_omega_trim) * dt;
    angle_q = (angle_accum >> OMEGA_SHIFT) & 0x0FFF;
}
