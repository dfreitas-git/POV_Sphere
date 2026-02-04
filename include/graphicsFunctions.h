
#pragma once

#include <Arduino.h>
#include <config.h>
#include <images.h>
#include <globals.h>
#include <math.h>



//#################################
// eyeball movement enum
//#################################
enum EYEBALL_MOVE { LEFT, RIGHT, UP, DOWN, CENTER };


//#################################
// Pinecrest owl structs
//#################################
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

struct OwlTimeline {
    uint32_t tSquawkOn;
    uint32_t tSquawkOff;
    uint32_t tBlinkOn;
    uint32_t tBlinkOff;
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


//#################################
// Color definitions
//#################################
// Struct to hold the rgb color fields
struct RGB { uint8_t r, g, b; };

// Struct to hold the color definitions
struct palette { 
    RGB white = {255,255,255};
    RGB gray  = {128,128,128};
    RGB mediumgray = {96,96,96};
    RGB darkgray  = {64,64,64};
    RGB red   = {255,0,0};
    RGB meduimred   = {128,0,0};
    RGB darkred   = {64,0,0};
    RGB green = {0,255,0};
    RGB mediumgreen = {0,128,0};
    RGB darkgreen = {0,64,0};
    RGB limegreen = {64,102,0};
    RGB olivegreen = {64,80,0};
    RGB blue  = {0,0,255};
    RGB mediumblue  = {0,0,128};
    RGB darkblue  = {0,0,64};
    RGB yellow = {255,150,0};
    RGB orange = {255,159,0};
    RGB pink   = {255,192,203}; 
    RGB purple = {128,0,128};
    RGB brown  = {102,51,64};
    RGB black  = {0,0,0};
};

constexpr RGB spiralColors[] = {
                {255,0,0},
                {0,255,0},
                {0,0,255},
              };


// Set up some pre-determined colors to cycle between (these give good contrast between each pair)
constexpr RGB colors[] = {
                {235,0,0},
                {0,235,0},
                {1,65,223},
                {10,229,0},
                {218,24,94},
                {74,160,240},
                {231,121,67},
                {72,106,16},
                {101,16,187},
                {5,17,37},
                {196,16,12},
                {0,235,235},
              };


// Struct to hold vertex point
struct Vec2 {
    int x, y;
};

// Counter that holds the number of times we ever tried to write a pixel outside the framebuffer bounds
static uint32_t outOfBoundsPixelCount = 0;

//#################################
// prototypes
//#################################
void writePixel(int col, int row, const struct RGB& color);
void drawSpiral(uint32_t phase, int K, int numSpirals, int thickness, const struct RGB& color);
void drawLetter(uint8_t (*letter)[7], int llX ,int llY, const struct RGB& bgColor, const struct RGB& fgColor);
void drawLine(int pt0X, int pt0Y, int pt1X, int pt1Y, int thickness, const struct RGB& color);
void drawArc(const Vec2& p1, const Vec2& p2, const Vec2& p3, int thickness, const struct RGB& color);
bool angleBetweenCCW(float a, float b, float c);
void writeArcPixels(int cx, int cy, int r, float theta, int thickness, RGB color);
void drawRect(int llX, int llY, int urX, int urY, int rotate, const struct RGB& color);
void drawQuad(int pt0X, int pt0Y, int pt1X, int pt1Y, int pt2X, int pt2Y, int pt3X, int pt3Y, int rotate, const struct RGB& color);
void drawCircle(int centerX, int centerY, int radius, const struct RGB& color);
void drawEllipse( int centerX, int centerY, int radiusX, int radiusY, float rotate, const struct RGB& color);
void drawDiamond(int centerX, int centerY, int extentX, int extentY, const struct RGB& color);
void drawTriangle(const Vec2& v1, const Vec2& v2, const Vec2& v3, const struct RGB& color);
float cross(const Vec2& a, const Vec2& b, const Vec2& c);
void drawGhost(int centerX, int centerY, const struct RGB& bodyColor, const struct RGB& bgColor, const struct RGB& eyeColor, bool floatUp);
bool renderBlink(uint32_t now);
void updateOwlEvents(uint32_t now, uint32_t sceneStartTime, const OwlEvent* timeline, size_t eventCount, size_t& owlNextEvent);
void drawOwl(int centerX, int centerY, bool blink, bool squawk);
void drawPacman(int centerX, int centerY, const struct RGB& bodyColor, const struct RGB& bgColor, bool mouthOpen);
void drawEyeball(int centerX, int centerY, int radius, const struct RGB& eyeColor, const struct RGB& bgColor, const struct RGB& fgColor, int move );
void fillBB_pacman();
void fillBB_pacman1();
void fillBB_checker();
void fillBB_fade();
void fillBB_paint();
void fillBB_hBands();
void fillBB_image();
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

//#################################
//  Math functions 
//#################################

// Sine for each column (120 total) by look-up-table for speed
constexpr float sineLUT[COLUMNS] = {
     0.0,.052,.105,.156,.208,.259,.309,.358,.407,.454,.500,.545,.588,.629,.669,.707,.749,.777,.809,.839,.866,.891,.914,.934,.951,.966,.978,.988,.995,.999, 
     1.0,.999,.995,.988,.978,.966,.951,.934,.914,.891,.866,.839,.809,.777,.749,.707,.669,.629,.588,.545,.500,.454,.407,.358,.309,.259,.208,.156,.105,.052,
     0.0,-.052,-.105,-.156,-.208,-.259,-.309,-.358,-.407,-.454,-.500,-.545,-.588,-.629,-.669,-.707,-.749,-.777,-.809,-.839,-.866,-.891,-.914,-.934,-.951,-.966,-.978,-.988,-.995,-.999,
    -1.0,-.999,-.995,-.988,-.978,-.966,-.951,-.934,-.914,-.891,-.866,-.839,-.809,-.777,-.749,-.707,-.669,-.629,-.588,-.545,-.500,-.454,-.407,-.358,-.309,-.259,-.208,-.156,-.105,-.052
};

//#################################
// Fonts used by graphicsFunctions
//#################################
struct font_7x5 { 
    uint8_t P[7][5] = {{0,0,0,0,0},
                       {0,1,1,0,0},
                       {0,1,0,1,0},
                       {0,1,1,0,0},
                       {0,1,0,0,0},
                       {0,1,0,0,0},
                       {0,0,0,0,0},
                      };
    uint8_t I[7][5] = {{0,0,0,0,0},
                       {0,1,1,1,0},
                       {0,0,1,0,0},
                       {0,0,1,0,0},
                       {0,0,1,0,0},
                       {0,1,1,1,0},
                       {0,0,0,0,0},
                      };
    uint8_t N[7][5] = {{0,0,0,0,0},
                       {0,1,1,1,0},
                       {0,1,0,1,0},
                       {0,1,0,1,0},
                       {0,1,0,1,0},
                       {0,1,0,1,0},
                       {0,0,0,0,0},
                      };
    uint8_t E[7][5] = {{0,0,0,0,0},
                       {0,1,1,1,0},
                       {0,1,0,0,0},
                       {0,1,1,0,0},
                       {0,1,0,0,0},
                       {0,1,1,1,0},
                       {0,0,0,0,0},
                      };
    uint8_t C[7][5] = {{0,0,0,0,0},
                       {0,0,1,1,0},
                       {0,1,0,0,0},
                       {0,1,0,0,0},
                       {0,1,0,0,0},
                       {0,0,1,1,0},
                       {0,0,0,0,0},
                      };
    uint8_t R[7][5] = {{0,0,0,0,0},
                       {0,1,1,0,0},
                       {0,1,0,1,0},
                       {0,1,1,0,0},
                       {0,1,1,0,0},
                       {0,1,0,1,0},
                       {0,0,0,0,0},
                      };
    uint8_t S[7][5] = {{0,0,0,0,0},
                       {0,0,1,1,0},
                       {0,1,0,0,0},
                       {0,0,1,0,0},
                       {0,0,0,1,0},
                       {0,1,1,0,0},
                       {0,0,0,0,0},
                      };
    uint8_t T[7][5] = {{0,0,0,0,0},
                       {0,1,1,1,0},
                       {0,0,1,0,0},
                       {0,0,1,0,0},
                       {0,0,1,0,0},
                       {0,0,1,0,0},
                       {0,0,0,0,0},
                      };
    uint8_t N2[7][5] = {{0,0,0,0,0},
                        {0,1,1,0,0},
                        {0,0,0,1,0},
                        {0,0,1,0,0},
                        {0,1,0,0,0},
                        {0,1,1,1,0},
                        {0,0,0,0,0},
                      };
    uint8_t N0[7][5] = {{0,0,0,0,0},
                        {0,1,1,1,0},
                        {0,1,0,1,0},
                        {0,1,0,1,0},
                        {0,1,0,1,0},
                        {0,1,1,1,0},
                        {0,0,0,0,0},
                      };
    uint8_t N6[7][5] = {{0,0,0,0,0},
                        {0,0,1,0,0},
                        {0,1,0,0,0},
                        {0,1,1,0,0},
                        {0,1,0,1,0},
                        {0,0,1,0,0},
                        {0,0,0,0,0},
                      };
};

struct font_7x7 { 
    uint8_t P[7][7] = {{0,0,0,0,0,0,0},
                       {0,1,1,1,1,0,0},
                       {0,1,0,0,0,1,0},
                       {0,1,1,1,1,0,0},
                       {0,1,0,0,0,0,0},
                       {0,1,0,0,0,0,0},
                       {0,1,0,0,0,0,0},
                      };
    uint8_t I[7][7] = {{0,0,0,0,0,0,0},
                       {0,1,1,1,1,1,0},
                       {0,0,0,1,0,0,0},
                       {0,0,0,1,0,0,0},
                       {0,0,0,1,0,0,0},
                       {0,0,0,1,0,0,0},
                       {0,1,1,1,1,1,0},
                      };
    uint8_t N[7][7] = {{0,0,0,0,0,0,0},
                       {0,1,0,0,0,1,0},
                       {0,1,1,0,0,1,0},
                       {0,1,0,1,0,1,0},
                       {0,1,0,0,1,1,0},
                       {0,1,0,0,0,1,0},
                       {0,1,0,0,0,1,0},
                      };
    uint8_t E[7][7] = {{0,0,0,0,0,0,0},
                       {0,1,1,1,1,1,0},
                       {0,1,0,0,0,0,0},
                       {0,1,1,1,0,0,0},
                       {0,1,0,0,0,0,0},
                       {0,1,0,0,0,0,0},
                       {0,1,1,1,1,1,0},
                      };
    uint8_t C[7][7] = {{0,0,0,0,0,0,0},
                       {0,0,1,1,1,1,0},
                       {0,1,0,0,0,0,0},
                       {0,1,0,0,0,0,0},
                       {0,1,0,0,0,0,0},
                       {0,1,0,0,0,0,0},
                       {0,0,1,1,1,1,0},
                      };
    uint8_t R[7][7] = {{0,0,0,0,0,0,0},
                       {0,1,1,1,1,0,0},
                       {0,1,0,0,0,1,0},
                       {0,1,1,1,1,0,0},
                       {0,1,0,1,1,0,0},
                       {0,1,0,0,0,1,0},
                       {0,1,0,0,0,1,0},
                      };
    uint8_t S[7][7] = {{0,0,0,0,0,0,0},
                       {0,0,1,1,1,1,0},
                       {0,1,0,0,0,0,0},
                       {0,0,1,1,1,0,0},
                       {0,0,0,0,0,1,0},
                       {0,0,0,0,0,1,0},
                       {0,1,1,1,1,0,0},
                      };
    uint8_t T[7][7] = {{0,0,0,0,0,0,0},
                       {0,1,1,1,1,1,0},
                       {0,0,0,1,0,0,0},
                       {0,0,0,1,0,0,0},
                       {0,0,0,1,0,0,0},
                       {0,0,0,1,0,0,0},
                       {0,0,0,1,0,0,0},
                      };
    uint8_t N2[7][7] = {{0,0,0,0,0,0,0},
                        {0,0,1,1,1,0,0},
                        {0,0,0,0,0,1,0},
                        {0,0,0,0,1,0,0},
                        {0,0,0,1,0,0,0},
                        {0,0,1,0,0,0,0},
                        {0,1,1,1,1,0,0},
                      };
    uint8_t N0[7][7] = {{0,0,0,0,0,0,0},
                        {0,0,1,1,1,0,0},
                        {0,1,0,0,0,1,0},
                        {0,1,0,0,0,1,0},
                        {0,1,0,0,0,1,0},
                        {0,1,0,0,0,1,0},
                        {0,0,1,1,1,0,0},
                      };
    uint8_t N6[7][7] = {{0,0,0,0,0,0,0},
                        {0,0,1,0,0,0,0},
                        {0,1,0,0,0,0,0},
                        {0,1,1,1,1,0,0},
                        {0,1,0,0,0,1,0},
                        {0,1,0,0,0,1,0},
                        {0,0,1,1,1,0,0},
                      };
};
