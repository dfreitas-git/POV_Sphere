
#pragma once

#include <Arduino.h>
#include <renderTypes.h>
#include <globals.h>
#include <colors.h>
#include <fonts.h>

struct MarshmallowState;

class GraphicsAnimations {

  public:
    void clearBackground(FrameBuffer bbuf);
    void initShootingStars();
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
    void renderMarshmallow(FrameBuffer bbuf, MarshmallowState& mm,uint32_t now, palette& colors);
    void fadeFramebuffer(FrameBuffer bbuf, uint8_t decay);
    void meridians(FrameBuffer bbuf);

  private:
    float lerp(float a, float b, float f);
    void  lerpColor(RGB& out, RGB& a, RGB& b, float f);
    float clamp(float val, float minVal, float maxVal);
    void shiftDown(FrameBuffer bbuf);
    void renderRaw(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderToasting(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderMelting(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderDropping(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderBurnt(MarshmallowState& mm, uint32_t t, palette& colors);
    void renderSmoke(MarshmallowState& mm, uint32_t t, palette& colors);
};