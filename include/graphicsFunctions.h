
#pragma once

#include <Arduino.h>
#include <config.h>
#include <images.h>
#include <globals.h>
#include <math.h>

// Struct to hold the rgb color fields
struct RGB { uint8_t r, g, b; };

// Struct to hold vertex point
struct Vec2 {
    float x, y;
};

void fillBB_spiral();
void writePixel(uint8_t row, uint8_t col, const struct RGB& color);
void drawSpiral(uint32_t phase, int K, uint8_t numSpirals, uint8_t thickness, const struct RGB& color);
void drawRect(uint8_t llX, uint8_t llY, uint8_t urX, uint8_t urY, const struct RGB& color);
void drawCircle(uint8_t centerX, uint8_t centerY, uint8_t radius, const struct RGB& color);
void drawOval(uint8_t centerX, uint8_t centerY, uint8_t radiusA, uint8_t radiusB, const struct RGB& color);
void drawTriangle(const Vec2& v1, const Vec2& v2, const Vec2& v3, const struct RGB& color);
void drawArc(const Vec2& p1, const Vec2& p2, const uint8_t radius, const struct RGB& color);
float cross(const Vec2& a, const Vec2& b, const Vec2& c);
void fillBB_pacman();
void fillBB_checker();
void fillBB_fade();
void fillBB_paint();
void fillBB_hBands();
void fillBB_hFade();
void fillBB_vFade();
void fillBB_image();
void fillBB_fade();
void fillBB_paint();
void fillBB_hBands();
void fillBB_hFade();
void fillBB_vFade();
void fillBB_image();

// Sine for each column via look-up-table for speed
constexpr float sineLUT[COLUMNS] = {
     0.0,.052,.105,.156,.208,.259,.309,.358,.407,.454,.500,.545,.588,.629,.669,.707,.749,.777,.809,.839,.866,.891,.914,.934,.951,.966,.978,.988,.995,.999, 
     1.0,.999,.995,.988,.978,.966,.951,.934,.914,.891,.866,.839,.809,.777,.749,.707,.669,.629,.588,.545,.500,.454,.407,.358,.309,.259,.208,.156,.105,.052,
     0.0,-.052,-.105,-.156,-.208,-.259,-.309,-.358,-.407,-.454,-.500,-.545,-.588,-.629,-.669,-.707,-.749,-.777,-.809,-.839,-.866,-.891,-.914,-.934,-.951,-.966,-.978,-.988,-.995,-.999,
    -1.0,-.999,-.995,-.988,-.978,-.966,-.951,-.934,-.914,-.891,-.866,-.839,-.809,-.777,-.749,-.707,-.669,-.629,-.588,-.545,-.500,-.454,-.407,-.358,-.309,-.259,-.208,-.156,-.105,-.052
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