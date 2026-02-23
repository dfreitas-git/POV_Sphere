
#pragma once

#include <Arduino.h>

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


class GraphicsParticles {

public:
  //Star* getStars();
  // This is used in the star and fireworks animation.  Its the number of stars shooting at any given time
  Star stars[NUM_STARS];

private:

};