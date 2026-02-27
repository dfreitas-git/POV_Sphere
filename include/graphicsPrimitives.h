

#pragma once
#include <Arduino.h>
#include <renderTypes.h>
#include <globals.h>
#include <colors.h>


// Struct to hold vertex point
struct Vec2 {
    int x, y;
};

//#####################
//  Class Definition                         
//#####################
class GraphicsPrimitives {

public:
  void writePixel(FrameBuffer bbuf, int col, int row, const struct RGB& color);
  void drawLine(FrameBuffer bbuf, int pt0X, int pt0Y, int pt1X, int pt1Y, int thickness, const struct RGB& color);
  void drawTriangle(FrameBuffer bbuf, const Vec2& v1, const Vec2& v2, const Vec2& v3, const struct RGB& color);
  void drawArc(FrameBuffer bbuf,const Vec2& p1, const Vec2& p2, const Vec2& p3, int thickness, const struct RGB& color);
  void drawRect(FrameBuffer bbuf, int llX, int llY, int urX, int urY, int rotate, const struct RGB& color);
  void drawQuad(FrameBuffer bbuf, int pt0X, int pt0Y, int pt1X, int pt1Y, int pt2X, int pt2Y, int pt3X, int pt3Y, int rotate, const struct RGB& color);
  void drawDiamond(FrameBuffer bbuf,int centerX, int centerY, int extentX, int extentY, const struct RGB& color);
  void drawEllipse(FrameBuffer bbuf, int centerX, int centerY, int radiusX, int radiusY, float rotateDeg, const struct RGB& color);
  void drawCircle(FrameBuffer bbuf, int centerX, int centerY, int radius, const struct RGB& color);
  void drawSpiral(FrameBuffer bbuf,uint32_t phase, int K, int numSpirals, int thickness, int drawToRow, const struct RGB& color);
  void drawChar(FrameBuffer bbuf, char c, int x, int y, RGB fg, RGB bg);
  void drawString(FrameBuffer bbuf, const char* s, int x, int y, RGB fg, RGB bg);

private:
  float cross(const Vec2& a, const Vec2& b, const Vec2& c);
  bool pointInTriangle(const Vec2& p, const Vec2& v1, const Vec2& v2, const Vec2& v3);
  bool angleBetweenCCW(float a, float b, float c);
  void writeArcPixels(FrameBuffer bbuf, int cx, int cy, int r, float theta, int thickness, RGB color);
   int wrapDist(int a, int b);

// Counter that holds the number of times we ever tried to write a pixel outside the framebuffer bounds
uint32_t outOfBoundsPixelCount = 0;

};
