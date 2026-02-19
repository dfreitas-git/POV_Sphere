
#pragma once

#include <Arduino.h>
#include <config.h>
#include <renderer.h>
#include <images.h>
#include <globals.h>
#include <math.h>
#include <renderer.h>
#include <graphicsPrimitives.h>
#include <graphicsComposites.h>
#include <fonts.h>
#include <colors.h>

extern Renderer renderer;
extern GraphicsPrimitives gPrim;
extern GraphicsComposites gComp;


//#################################
// Pinecrest campground animation
//#################################

// owl structs
const uint32_t BLINK_DURATION_MS = 200;   // visible owl blink

enum OwlEventType {
  OWL_BLINK,
  OWL_SQUAWK_ON,
  OWL_SQUAWK_OFF
};

struct OwlEvent {
  uint32_t timeMs;     // relative to scene start
  OwlEventType type;
};

struct OwlAnimState {
  bool squawking;
  bool blinkActive;
  uint32_t blinkStartTime;
  uint32_t squawkStartTime;
};

static OwlAnimState owl = {
  .squawking = false,
  .blinkActive = false,
  .blinkStartTime = 0,
  .squawkStartTime = 0
};


// Marshmallow structs
// These are the states that are edge triggeed by the clock
enum MarshmallowEventType {
  MM_START_ROASTING,
  MM_START_TOASTING,
  MM_START_MELTING,
  MM_START_DROPPING,
  MM_START_BURNING,
  MM_START_SMOKING
};

// Each triggered state puts us into a render state where the actual graphics are generated
enum MarshmallowPhase {
  MM_RAW,
  MM_TOASTING,
  MM_MELTING,
  MM_DROPPING,
  MM_BURNT,
  MM_SMOKE
};

// This is the struct filled in the main graphicsFunctions.cpp file that describes the Pinecrest scene
struct MarshmallowEvent {
  uint32_t timeMs;       // relative to scene start
  MarshmallowEventType type;
  uint32_t phaseTimeMs;  // relative to event start.  How long any phase lerping happens
};

// This is the struct that is updated based on the state's phase.  It's how we control motion transforms 
// while rendering for a particular phase.
struct MarshmallowState {
  MarshmallowPhase phase;
  uint32_t phaseStartTime;
  uint32_t phaseRunTime;

  // base pose
  int cx, cy;
  int rotation;
  int halfSize;
  RGB color;
};

// Initial state/phase at the beginning of the scene
static MarshmallowState  mm = {
  .phase = MM_RAW,
  .phaseStartTime = 0,
  .cx = 70, .cy = 26,
  .rotation = 0,
  .halfSize = 3,
  .color = {0,0,0}
};

// Struct for the shootingStar animation
#define NUM_STARS 20

typedef struct {
  float x;
  float y;
  float vx;
  float vy;
  uint8_t r, g, b;
  uint16_t age;
  uint16_t maxAge;
  bool active;
} Star;


//#################################
// prototypes
//#################################
float lerp(float a, float b, float f);
float clamp(float val, float minVal, float maxVal);
void initShootingStars();
void initSparkShower();
void initRocket();
void initExplosion(float x, float y);
void cycle_spiral(int numSpirals, int thickness, int twist);
bool renderBlink(uint32_t now);
void updateOwlEvents(uint32_t now, uint32_t sceneStartTime, const OwlEvent* timeline, size_t eventCount, size_t& owlNextEvent);
void fillBB_pacman();
void fillBB_pacman1();
void fillBB_checker();
void fillBB_fade();
void fillBB_paint();
void fillBB_hBands();
void fillBB_hFade();
void fillBB_vFade();
void fillBB_diamond();
void fillBB_spiralR();
void fillBB_spiralL();
void fillBB_spiralD();
void fillBB_eyeball();
void fillBB_image();
void fillBB_eyeball();
void fillBB_pinecrest();
void fillBB_shootingStar();
void fillBB_sparkShower();
void fillBB_fireworks();

//#################################
//  Math functions 
//#################################

// Sine for each column (120 total) by look-up-table for speed.  Use this if the regular sin or sinf are too slow.
constexpr float sineLUT[COLUMNS] = {
     0.0,.052,.105,.156,.208,.259,.309,.358,.407,.454,.500,.545,.588,.629,.669,.707,.749,.777,.809,.839,.866,.891,.914,.934,.951,.966,.978,.988,.995,.999, 
     1.0,.999,.995,.988,.978,.966,.951,.934,.914,.891,.866,.839,.809,.777,.749,.707,.669,.629,.588,.545,.500,.454,.407,.358,.309,.259,.208,.156,.105,.052,
     0.0,-.052,-.105,-.156,-.208,-.259,-.309,-.358,-.407,-.454,-.500,-.545,-.588,-.629,-.669,-.707,-.749,-.777,-.809,-.839,-.866,-.891,-.914,-.934,-.951,-.966,-.978,-.988,-.995,-.999,
    -1.0,-.999,-.995,-.988,-.978,-.966,-.951,-.934,-.914,-.891,-.866,-.839,-.809,-.777,-.749,-.707,-.669,-.629,-.588,-.545,-.500,-.454,-.407,-.358,-.309,-.259,-.208,-.156,-.105,-.052
};
