
#pragma once

#include <Arduino.h>
#include <renderTypes.h>
#include <globals.h>
#include <colors.h>
#include <fonts.h>

extern void fadeWrapper(FrameBuffer bbuf);
extern void hFadeWrapper(FrameBuffer bbuf);
extern void vFadeWrapper(FrameBuffer bbuf);
extern void hBandsWrapper(FrameBuffer bbuf);
extern void paintWrapper(FrameBuffer bbuf);
extern void checkerWrapper(FrameBuffer bbuf);
extern void spiralRWrapper(FrameBuffer bbuf);
extern void spiralLWrapper(FrameBuffer bbuf);
extern void spiralDWrapper(FrameBuffer bbuf);
extern void diamondWrapper(FrameBuffer bbuf);
extern void flowerWrapper(FrameBuffer bbuf);
extern void eyeballWrapper(FrameBuffer bbuf);
extern void pinecrestWrapper(FrameBuffer bbuf);
extern void pacmanWrapper(FrameBuffer bbuf);
extern void pacman1Wrapper(FrameBuffer bbuf);
extern void shootingStarWrapper(FrameBuffer bbuf);
extern void sparkShowerWrapper(FrameBuffer bbuf);
extern void fireworksWrapper(FrameBuffer bbuf);


// Pinecrest campground animation structs
// owl structs
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

  // State variables for the fireworks animation.  ROCKET is for the single rising streak, EXPLOSION is when the "spokes" shoot out
  typedef enum {
    FIREWORK_ROCKET,
    FIREWORK_EXPLOSION
  } FireworkState;

class GraphicsAnimations {

public:
  void initShootingStars();
  void initRocket();
  void fade(FrameBuffer bbuf);
  void hFade(FrameBuffer bbuf);
  void vFade(FrameBuffer bbuf);
  void hBands(FrameBuffer bbuf);
  void paint(FrameBuffer bbuf);
  void checker(FrameBuffer bbuf);
  void cycle_spiral(FrameBuffer bbuf, int numSpirals, int thickness, int twist);
  void spiralR(FrameBuffer bbuf);
  void spiralL(FrameBuffer bbuf);
  void spiralD(FrameBuffer bbuf);
  void diamond(FrameBuffer bbuf);
  void flower(FrameBuffer bbuf);
  void eyeball(FrameBuffer bbuf);
  bool renderBlink(uint32_t now);
  void renderRaw(uint32_t t, palette& colors);
  void renderToasting(uint32_t t, palette& colors);
  void renderMelting(uint32_t t, palette& colors);
  void renderDropping(uint32_t t, palette& colors);
  void renderBurnt(uint32_t t, palette& colors);
  void renderSmoke(uint32_t t, palette& colors);
  void updateMarshmallowEvents(uint32_t now, uint32_t sceneStartTime, const MarshmallowEvent* mmTimeline, size_t eventCount, size_t& mmNextEvent, palette& colors);
  void renderMarshmallow(uint32_t now, palette& colors);
  void updateOwlEvents(uint32_t now, uint32_t sceneStartTime, const OwlEvent* timeline, size_t eventCount, size_t& owlNextEvent);
  void pinecrest(FrameBuffer bbuf);
  void pacman(FrameBuffer bbuf);
  void pacman1(FrameBuffer bbuf);
  void shootingStar(FrameBuffer bbuf);
  void sparkShower(FrameBuffer bbuf);
  void fireworks(FrameBuffer bbuf);

private:
  float lerp(float a, float b, float f);
  void  lerpColor(RGB& out, RGB& a, RGB& b, float f);
  float clamp(float val, float minVal, float maxVal);
  void fadeFramebuffer(uint8_t decay);
  void writeHead(int col, int row, RGB color);
  void initExplosion(float x, float y);
  void shiftDown(FrameBuffer bbuf);


};

extern GraphicsAnimations gAnim;

// wrapper
void fadeWrapper(FrameBuffer buf);
void hFadeWrapper(FrameBuffer bbuf);
void vFadeWrapper(FrameBuffer bbuf);
void hBandsWrapper(FrameBuffer bbuf);
void paintWrapper(FrameBuffer bbuf);
void checkerWrapper(FrameBuffer bbuf);
void spiralRWrapper(FrameBuffer bbuf);
void spiralLWrapper(FrameBuffer bbuf);
void spiralDWrapper(FrameBuffer bbuf);
void diamondWrapper(FrameBuffer bbuf);
void flowerWrapper(FrameBuffer bbuf);
void eyeballWrapper(FrameBuffer bbuf);
void pinecrestWrapper(FrameBuffer bbuf);
void pacmanWrapper(FrameBuffer bbuf);
void pacman1Wrapper(FrameBuffer bbuf);
void shootingStarWrapper(FrameBuffer bbuf);
void sparkShowerWrapper(FrameBuffer bbuf);
void fireworksWrapper(FrameBuffer bbuf);