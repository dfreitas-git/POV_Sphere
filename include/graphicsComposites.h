
#pragma once

#include <Arduino.h>
#include <renderTypes.h>
#include <colors.h>

// eyeball movement enum
enum EYEBALL_MOVE { LEFT, RIGHT, UP, DOWN, CENTER };

//#####################
//  Class Definition                         
//#####################
class GraphicsComposites {

public:
  void drawOwl(FrameBuffer buf, int centerX, int centerY,  bool blink, bool squawk);
  void drawEyeball(FrameBuffer bbuf, int centerX, int centerY, int radius, const struct RGB& eyeColor, const struct RGB& bgColor, const struct RGB& fgColor, int move );
  void drawGhost(FrameBuffer bbuf, int centerX, int centerY, const struct RGB& bodyColor, const struct RGB& bgColor, const struct RGB& eyeColor, bool jumpUp);
  void drawPacman(FrameBuffer bbuf,int centerX, int centerY, const struct RGB& bodyColor, const struct RGB& bgColor, bool mouthOpen);

private:


};
