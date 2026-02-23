
#pragma once

#include <Arduino.h>
#include <colors.h>
#include <renderTypes.h>


// State variables for the fireworks animation.  ROCKET is for the single rising streak, EXPLOSION is when the "spokes" shoot out
typedef enum {
  FIREWORK_ROCKET,
  FIREWORK_EXPLOSION
} FireworkState;

// Each triggered state puts us into a render state where the actual graphics are generated
enum MarshmallowPhase {
  MM_RAW,
  MM_TOASTING,
  MM_MELTING,
  MM_DROPPING,
  MM_BURNT,
  MM_SMOKE
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

class GraphicsScenes {

public:
  GraphicsScenes();  // Constructor
  void pinecrest(FrameBuffer bbuf);
  void fireworks(FrameBuffer bbuf);
  void initRocket();
  void initExplosion(float x, float y);

private:

  // State var for the fireworks animation
  FireworkState fwState = FIREWORK_ROCKET;
  uint32_t stateStartTime = 0;

  MarshmallowState mm;  


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

  OwlAnimState owl = {
    .squawking = false,
    .blinkActive = false,
    .blinkStartTime = 0,
    .squawkStartTime = 0
  };

  // decide blink state based on time
  bool isBlinkActive(uint32_t now);
  

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


  // This is the struct filled in the main graphicsFunctions.cpp file that describes the Pinecrest scene
  struct MarshmallowEvent {
    uint32_t timeMs;       // relative to scene start
    MarshmallowEventType type;
    uint32_t phaseTimeMs;  // relative to event start.  How long any phase lerping happens
  };
  
  void updateOwlEvents(uint32_t now, uint32_t sceneStartTime, const OwlEvent* timeline, size_t eventCount, size_t& owlNextEvent);
  void updateMarshmallowEvents(uint32_t now, uint32_t sceneStartTime, const MarshmallowEvent* mmTimeline, size_t eventCount, size_t& mmNextEvent, palette& colors);

};