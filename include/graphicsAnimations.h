
#pragma once

#include <Arduino.h>
#include <renderTypes.h>
#include <globals.h>
#include <colors.h>
#include <fonts.h>

struct MarshmallowState;

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
    void clearBackground(FrameBuffer bbuf);
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
    void pacman(FrameBuffer bbuf);
    void pacman1(FrameBuffer bbuf);
    void shootingStar(FrameBuffer bbuf);
    void sparkShower(FrameBuffer bbuf);
    void fireworks(FrameBuffer bbuf);
    void renderMarshmallow(FrameBuffer bbuf, MarshmallowState& mm,uint32_t now, palette& colors);

  private:
    float lerp(float a, float b, float f);
    void  lerpColor(RGB& out, RGB& a, RGB& b, float f);
    float clamp(float val, float minVal, float maxVal);
    void fadeFramebuffer(FrameBuffer bbuf, uint8_t decay);
    void writeHead(FrameBuffer bbuf, int col, int row, RGB color);
    void initExplosion(float x, float y);
    void shiftDown(FrameBuffer bbuf);
    void renderRaw(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderToasting(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderMelting(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderDropping(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderBurnt(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderSmoke(MarshmallowState& mm, uint32_t t, palette& colors);
};