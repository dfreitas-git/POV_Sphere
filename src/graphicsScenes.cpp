
#include <graphicsComposites.h>
#include <graphicsAnimations.h>
#include <graphicsScenes.h>
#include <graphicsGlobals.h>

// Animation Scenes.  Contains just animation, composite placements, state, and timing

OwlAnimState owl = {
  .squawking = false,
  .blinkActive = false,
  .blinkStartTime = 0,
  .squawkStartTime = 0
};

// Initial state/phase at the beginning of the scene
MarshmallowState  mm = {
  .phase = MM_RAW,
  .phaseStartTime = 0,
  .cx = 70, .cy = 26,
  .rotation = 0,
  .halfSize = 3,
  .color = {0,0,0}
};


//#########  Pinecrest Edge based state update based on what time we have reached
void GraphicsScenes::updateOwlEvents(uint32_t now, uint32_t sceneStartTime, const OwlEvent* timeline, size_t eventCount, size_t& owlNextEvent) {

  uint32_t sceneElapsed = now - sceneStartTime;
  
  while (owlNextEvent < eventCount && sceneElapsed >= timeline[owlNextEvent].timeMs) {
    switch (timeline[owlNextEvent].type) {
      case OWL_BLINK:
        owl.blinkActive = true;
        owl.blinkStartTime = now;
        break;
  
      case OWL_SQUAWK_ON:
        owl.squawking = true;
        owl.squawkStartTime = now;
        break;
  
      case OWL_SQUAWK_OFF:
        owl.squawking = false;
        break;
    }
    owlNextEvent++;
  }
}

//#########  Edge triggered transition state change based on what time we reach
void GraphicsScenes::updateMarshmallowEvents(uint32_t now, uint32_t sceneStartTime, const MarshmallowEvent* mmTimeline, size_t eventCount, size_t& mmNextEvent, palette& colors) {

  uint32_t sceneElapsed = now - sceneStartTime;

  while (mmNextEvent < eventCount && sceneElapsed >= mmTimeline[mmNextEvent].timeMs) {
    switch (mmTimeline[mmNextEvent].type) {
        case MM_START_ROASTING:
          mm.phase = MM_RAW;
          mm.phaseStartTime = now;
          mm.phaseRunTime = mmTimeline[mmNextEvent].phaseTimeMs;
        break;

        case MM_START_TOASTING:
          mm.phase = MM_TOASTING;
          mm.phaseStartTime = now;
          mm.phaseRunTime = mmTimeline[mmNextEvent].phaseTimeMs;
        break;

        case MM_START_MELTING:
          mm.phase = MM_MELTING;
          mm.phaseStartTime = now;
          mm.phaseRunTime = mmTimeline[mmNextEvent].phaseTimeMs;
        break;

        case MM_START_DROPPING:
          mm.phase = MM_DROPPING;
          mm.phaseStartTime = now;
          mm.phaseRunTime = mmTimeline[mmNextEvent].phaseTimeMs;
        break;

        case MM_START_BURNING:
          mm.phase = MM_BURNT;
          mm.phaseStartTime = now;
          mm.phaseRunTime = mmTimeline[mmNextEvent].phaseTimeMs;
        break;

        case MM_START_SMOKING:
          mm.phase = MM_SMOKE;
          mm.phaseStartTime = now;
          mm.phaseRunTime = mmTimeline[mmNextEvent].phaseTimeMs;
        break;
    }
    mmNextEvent++;
  }
}

//#############################
//  Animated Pincrest Scene
//#############################
void GraphicsScenes::pinecrest(FrameBuffer bbuf) {

  // Get the colors
  static palette c;

  static uint32_t sceneStartTime = 0;
  static bool timelineInitialized = false;

  // Use time to control the animation
  uint32_t now = millis();

  // Start the scene time
  if (!timelineInitialized) {
    sceneStartTime = now;
    timelineInitialized = true;
  }

  // Clear the sphere with black
  gAnim.clearBackground(bbuf);

  // Draw the campfire elements
  gComp.drawCampfire(bbuf,75,7);
  gComp.drawRoastingStick(bbuf,80,22);

  // The marshmallow
  // define what time events should trigger when and any phase/lerp time for transitions
  const MarshmallowEvent mmTimeline[] = {
    { 1000, MM_START_ROASTING, 0 },
    { 2500, MM_START_TOASTING, 2000 },
    { 4500, MM_START_MELTING, 3000 },
    { 7500, MM_START_DROPPING, 1000 },
    { 8500, MM_START_BURNING, 1000 },
    { 9500, MM_START_SMOKING, 1500 },
  };
  constexpr size_t MM_EVENT_COUNT = sizeof(mmTimeline) / sizeof(mmTimeline[0]);

  // Update marshmallow events state based on time (edge triggered, not time windows)
  static size_t mmNextEvent = 0;
  updateMarshmallowEvents(now, sceneStartTime,mmTimeline,MM_EVENT_COUNT, mmNextEvent,c);

  // Now draw the marshmallow given the state and time we are at
  gAnim.renderMarshmallow(now, c);

  // the owl
  // define what time events should trigger when
  const OwlEvent owlTimeline[] = {
    { 4000, OWL_BLINK },
    { 8000, OWL_SQUAWK_ON },
    { 9500, OWL_SQUAWK_OFF },
    { 11000, OWL_BLINK },
  };
  constexpr size_t OWL_EVENT_COUNT = sizeof(owlTimeline) / sizeof(owlTimeline[0]);

  // Update owl events state based on time (edge triggered, not time windows)
  static size_t owlNextEvent = 0;
  updateOwlEvents(now, sceneStartTime,owlTimeline,OWL_EVENT_COUNT, owlNextEvent);

  // Now draw the owl given the state and time we are at
  bool blink  = gAnim.renderBlink(now);
  bool squawk = owl.squawking;
  gComp.drawOwl(bbuf,100, 25, blink, squawk);

  // PINECREST letters
  gComp.pinecrestLetters(bbuf,0,30);

  // Reset the scene
  if(now > sceneStartTime + 11500) {
    timelineInitialized = false;
    owlNextEvent = 0;
    mmNextEvent = 0;
  }
}