// POV_DotStar_DMA.ino
// ESP32 Arduino sketch: SK9822 (DotStar) driven with DMA, double-buffered,
// 4x-per-revolution, and one SPI channel serially multiplexing the DMA streams
// to the dotStar strips.
//
// Hardware assumptions:
// - ESP32 using core-0 for the slow stuff: UI, menu, rotary-switch inputs, motor control.  
//               Within core-0 we set up three freeRTOS tasks: graphics, motor, and UI,  
//               Each task has its own loop time. highest priority- motor. Middle priority- graphics
//               lowest priority - UI/rotary-encoder.
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
// Algorithmic Image generation:
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

// My local modules
#include <config.h>
#include <globals.h>
#include <graphicsFunctions.h>
#include <renderer.h>
#include <Adafruit_SSD1306.h>
#include <ClickEncoder.h>
#include <Ticker.h>
#include <stdint.h>
#include <images.h>
#include <motor.h>
#include <UI.h>


// uncomment this to print the stack size used for each of the freeRTOS tasks
//#define STACK_CHECK

// Prototypes
void uiTask(void* parameter) ;
void graphicsTask(void* parameter) ;
void motorTask(void* parameter) ;

// For generating sync pulses to measure timing using a scope
#define SYNC_PIN 26
#define SYNC_MASK (1UL << SYNC_PIN)

// use eventGroup to be sure setup is 100% complete before we start executing loop in core-0
EventGroupHandle_t initEvent;
#define INIT_DONE_BIT (1 << 0)

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

// for the core-0 task
TaskHandle_t uiTaskHandle;
TaskHandle_t graphicsTaskHandle;
TaskHandle_t motorTaskHandle;

Motor motor;
UI ui;


//#########################################
// Main Setup Code
//#########################################
void setup() {
  Serial.begin(115200);
  delay(1000);
 
  //Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());

  // Create a eventGroup for core-0 tasks to use to be sure setup is complete before starting their loops
  initEvent = xEventGroupCreate();

  // Pin setup
  pinMode(RING0_ENB, OUTPUT);
  pinMode(RING1_ENB, OUTPUT);
  pinMode(RING2_ENB, OUTPUT);
  pinMode(RING3_ENB, OUTPUT);

  // Sync pin for scope measurements
  pinMode(SYNC_PIN, OUTPUT);

  //Set up OLED on 2nd I2C bus
  Wire1.begin(SDA_OLED, SCL_OLED, 400000); // SDA, SCL, clock

  ui.begin();

  // Set up pwm
  motor.begin();
  ui.configureRPM(
    motor.getTargetRPM(),
    motor.getMinRPM(),
    motor.getMaxRPM()
  );

  // Uses default i2c interface to AS5600 to read angle
  //if (!as5600.begin()) {
  if (!motor.getSensor().begin()) {
    Serial.println("Could not find AS5600 sensor, check wiring!");
    while (1) {
      delay(10);
    }
  }
  Wire.setClock(800000);  // Speed up I2c to AS5600

  // Allocate frame buffers
  ensureBuffersAllocated();
  clearFrameBuffer(backBuffer);
  clearFrameBuffer(frontBuffer);

  // Need to initialize the shootingStar of fireworks seeds
  if(strcmp(imageTable[imageToDisplayIndex]->name ,"ShootingS") == 0) {
    initShootingStars();
  } else if(strcmp(imageTable[imageToDisplayIndex]->name ,"Fireworks") == 0) {
    initRocket();
  }

  // Initialize SPI with DMA
  initSpi();

  // Small stabilization time
  delay(10);

  // Get the angle the motor shaft is starting at and initialize the core-1 angle-accumulator/calculator
  //measuredAngle = as5600.getRawAngle(); 
  measuredAngle = motor.getSensor().getRawAngle();
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

  // Create task to run on core-0 for the UI code (lowest priority)
  xTaskCreatePinnedToCore(
    uiTask,
    "UI Task",
    3072,
    nullptr,
    1,
    &uiTaskHandle,
    0   //  Core 0
  );

  // Create task to run on core-0 for graphics generation and framebuffer filling (middle priority)
  xTaskCreatePinnedToCore(
    graphicsTask,
    "Graphics Task",
    3072,
    nullptr,
    2,
    &graphicsTaskHandle,
    0   //  Core 0
  );

  // Create task to run on core-0 for the motor/PLL control loop (highest priority)
  xTaskCreatePinnedToCore(
    motorTask,
    "Motor Task",
    3072,
    nullptr,
    3,
    &motorTaskHandle,
    0   //  Core 0
  );

  // We are done with setup.  Set the eventGroups
  xEventGroupSetBits(initEvent, INIT_DONE_BIT);
}


//###################################################################
// core-0 motor control/PLL loop.   Highest priority
//####################################################################
void motorTask(void* parameter) {

  // Be sure the setup section is complete before we start the core-0 loops
  xEventGroupWaitBits(
    initEvent,
    INIT_DONE_BIT,
    pdFALSE,   // don't clear
    pdTRUE,    // wait for all bits (just one here)
    portMAX_DELAY
  );
  
  // Loop time for sampling the motor angle, adjusting PLL
  const TickType_t period = pdMS_TO_TICKS(20); 
  TickType_t lastWake = xTaskGetTickCount();
  int pidDiv = 0; // Use to run the PID update at a slower rate
  constexpr uint8_t PID_DIVIDER = 5;

  for (;;) {
    // Periodically measure the shaft angle for motor rpm/PID control and core-1 PLL locking
    uint32_t now = micros();
    int angleDelta;

    // Go do a shaft angle measurement so core-1 angle-calculating "pll" can lock to it. 
    //measuredAngle = as5600.getRawAngle();
    measuredAngle = motor.getSensor().getRawAngle();

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

    lastAngle = measuredAngle;
    lastMeasuredTime = now;

    //  PID speed regulation
    if (++pidDiv >= PID_DIVIDER) {
      pidDiv = 0;
      float pwm = motor.updatePID(motorRPM);
  
      // See if the user wants the motor off
      if(motorOnOffFlag == 0) {
        ledcWrite(0, 0);
      } else {
        ledcWrite(0, (int)pwm);
      }
    }
    
   #ifdef STACK_CHECK 
    UBaseType_t watermark = uxTaskGetStackHighWaterMark(nullptr);
    Serial.printf("Motor stack free: %u words (%u bytes)\n",
                  watermark, watermark * 4);

    vTaskDelay(pdMS_TO_TICKS(1000));
   #endif 

    // Need to throttle the motor loop so other tasks can execute too
    vTaskDelayUntil(&lastWake, period); // scheduling point
  }
}

//###################################################################
// core-0 graphics generation, framebuffer filling.  Medium priority
//####################################################################
void graphicsTask(void* parameter) {

  // Be sure the setup section is complete before we start the core-0 loops
  xEventGroupWaitBits(
    initEvent,
    INIT_DONE_BIT,
    pdFALSE,   // don't clear
    pdTRUE,    // wait for all bits (just one here)
    portMAX_DELAY
  );

  const TickType_t framePeriod = pdMS_TO_TICKS(16); // ~60 Hz
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {

    // Rotates the image by shifting the index into the framebuffer
    uint32_t curMillis = millis();
    if(scrollOnOffFlag &&  curMillis - lastScrollTime > SCROLL_UPDATE_TIME) {

      // Shift where in the frame buffer we get the column to display.  Use this to scroll the image.
      framebufferOffset = (framebufferOffset - 1 + COLUMNS) % COLUMNS;
      lastScrollTime=curMillis;
    }

    // Global brightness: 0b111xxxxx (5-bit current control)
    // Read the brightness setting (can be changed in the OLED menu)  Map to 0-1F
    fiveBitBright = map(brightness,0,10,0,31);

    // Load the backBuffer with the next frame to display
    if (!backBufferFilled) {
      if(demoAll) {
        if(millis() - lastAnimateTime > DEMO_DISPLAY_TIME) {
          imageToDisplayIndex++;
          if(imageToDisplayIndex >= IMG_COUNT) {
            imageToDisplayIndex = 0;
          }
          lastAnimateTime = millis();
        }
      }
      if(previousImageToDisplayIndex != imageToDisplayIndex) {
        // need to flush the framebuffers if we are starting to display a new image 
        clearFrameBuffer(backBuffer);
        clearFrameBuffer(frontBuffer);
        if(strcmp(imageTable[imageToDisplayIndex]->name ,"ShootingS") == 0) {
          initShootingStars();
        } else if(strcmp(imageTable[imageToDisplayIndex]->name ,"Fireworks") == 0) {
          initRocket();
        }
        previousImageToDisplayIndex = imageToDisplayIndex;
      }

      imageTable[imageToDisplayIndex]->functionPtr();
      backBufferFilled = true;
    }

    #ifdef STACK_CHECK 
    UBaseType_t watermark = uxTaskGetStackHighWaterMark(nullptr);
    Serial.printf("Graphics stack free: %u words (%u bytes)\n",
                  watermark, watermark * 4);

    vTaskDelay(pdMS_TO_TICKS(1000));
    #endif

    // Need to throttle the task so other tasks have a chance to work
    vTaskDelayUntil(&lastWake, framePeriod);
  }
}

//###################################################################
// core-0 UI task.   Lowest priority
//####################################################################
void uiTask(void* parameter) {

  // Be sure the setup section is complete before we start the core-0 loops
  xEventGroupWaitBits(
  initEvent,
  INIT_DONE_BIT,
  pdFALSE,   // don't clear
  pdTRUE,    // wait for all bits (just one here)
  portMAX_DELAY
);

  for (;;) {

    // Get any user input and update the OLED
    ui.update();
    ui.drawMenu();

    #ifdef STACK_CHECK 
    UBaseType_t watermark = uxTaskGetStackHighWaterMark(nullptr);
    Serial.printf("UI stack free: %u words (%u bytes)\n",
                  watermark, watermark * 4);

    vTaskDelay(pdMS_TO_TICKS(1000));
    #endif

    //Serial.println("Updated OLED");
    // Need to throttle the UI loop so other tasks have time to execute
    vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz UI scan
  }
}


//#########################################################################
// Main Loop for core-1. Core-1 will run the critical POV data DMA loop.
//#########################################################################
void loop() {

  //sync_high();  // Debug rising edge to measure timing on a scope
  //sync_low();  // Debug falling edge to measure timing on a scope

  // Make a local copy of the shaft speed that core-0 measured.
  core_1_omega_ff = omega_ff;      // local copies used by core-1.
  core_1_omega_trim = omega_trim;     

  // Now compute the angle_q for the current time (i.e. what the Sphere angle currently is)
  motor.computeCurrentAngle();   

  uint16_t adjustedAngle = angle_q % AS5600_COUNTS;  // modulo to wrap result in case of overflow

  int32_t triggerPoint = adjustedAngle - nextColumnAngle;
  if(triggerPoint < -2048) triggerPoint += AS5600_COUNTS;
  if(triggerPoint >  2048) triggerPoint -= AS5600_COUNTS;

  // Once the current angle has reached the next column, trigger a DMA transfer if the backBuffer is ready with a new frame
  if(triggerPoint >= 0) {

    // Check to see how many columns the shaft advanced.  Should usually be 1, but if there was some CPU delay the shaft may have advanced further
    uint32_t columnsAdvanced = (triggerPoint * COLUMNS / AS5600_COUNTS) + 1;

    uint16_t prevColumn = columnIndex;
    columnIndex = (columnIndex + columnsAdvanced) % COLUMNS;
    nextColumnAngle = (columnIndex + 1) * (AS5600_COUNTS / COLUMNS);

    // Take care of angle wrapping from 360->0
    if (nextColumnAngle >= AS5600_COUNTS) {
      nextColumnAngle -= AS5600_COUNTS;
    }

    // When core-0 filles the backBuffer, swap it into the frontBuffer once we get 
    // back to column-0.  The renderer always renders to the sphere from the frontBuffer.
    bool wrapped = (columnIndex < prevColumn);
    if(wrapped && backBufferFilled) {
      swapBuffersAtomic();
     backBufferFilled = false;
    }
  
    // Go update the strips. 
    if(!dmaBusy) {
      updateColumnLEDs((columnIndex + framebufferOffset) % COLUMNS);
    }
  }
}
