
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
    bool renderBlink(uint32_t now);
    void renderRaw(uint32_t t, palette& colors);
    void renderToasting(uint32_t t, palette& colors);
    void renderMelting(uint32_t t, palette& colors);
    void renderDropping(uint32_t t, palette& colors);
    void renderBurnt(uint32_t t, palette& colors);
    void renderSmoke(uint32_t t, palette& colors);
    void renderMarshmallow(uint32_t now, palette& colors);
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

// wrapper
/*
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
*/