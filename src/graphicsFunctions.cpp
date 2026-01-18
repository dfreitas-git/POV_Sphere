
#include <Arduino.h>
#include <config.h>
#include <images.h>
#include <globals.h>
#include <graphicsFunctions.h>

// Each function can be selected by the user from the rotary-encoder/OLED interface
//
// The functions take the absolute time (millis()) and compute animation cycles, phase, 
// animation speed, etc. to compute pixel RGB values anywhere on the sphere.  Then we just
// iterate across the available rows/columns and compute those pixels (basically a coarse
// mapping of the sphere data to the sparse LED matrix overlayed onto it).


//#############################################
//  Function to display a fade on the Sphere
//  Slow fade in/out of solid colors
//  cycle increments once per full fade
//  phase (in radians) is always 0 … 2π within that cycle
//  Randomly switch colors at cycle boundaries
//  Brightness is just cos(phase) (shifted to 0-255 result)
//####################################
void fillBB_fade() {

  const float periodMs = 3000.0f;  // one full fade cycle

  // graphics are based on the actual time so frames are synced to the clock
  float t = millis();

  // What cycle are we in?
  uint32_t cycle = uint32_t(t / periodMs);
  static uint32_t lastCycle = 0;

  // Compute the phase angle (in radians 0-2pi) for the given time within the cycle
  float phase = (t - cycle * periodMs) * (2.0f * PI / periodMs);  

  // Brightness: [-1,1] -> [0,255]
  // Use cos so brightness peaks are in phase 
  //uint8_t bright = (1 - cos(phase)) * 0.5 * 255;
  float bright = (1 - cos(phase)) * 0.5;

  // Choose color deterministically
  //uint8_t r8 = 0, g8 = 0, b8 = 0;
  static uint8_t r8, g8, b8;
  static uint8_t r8Base, g8Base, b8Base;

  // Switches to the next color when the cycle count increments which is where cos(phase) is zero so we softly transition
  if(cycle != lastCycle) {
    lastCycle = cycle;
    r8Base = uint8_t(random(256));
    g8Base = uint8_t(random(256));
    b8Base = uint8_t(random(256));
  }
  r8 = r8Base * bright;
  g8 = g8Base * bright;
  b8 = b8Base * bright;

  for (uint8_t col = 0; col < COLUMNS; col++) {
    for (uint8_t row = 0; row < ROWS; row++) {
      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = r8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
    }
  }
}


//###############################################################
//  Color flowing out the top, down the Sphere in dripping sheets
//  like paint poured on top of the Sphere.
//###############################################################
void fillBB_paint() {

  // Create a head pointer which is the leading edge of the first spill color
  uint16_t spillTime_mS = 4000;   // Time to iterate acroll 48 rows
  uint32_t elapsed = millis();
  uint32_t head = (elapsed * 48) / spillTime_mS;   // What row the spill edge is at
  uint32_t cycle = elapsed / spillTime_mS;        // integer cycle count  
  head %= 48;

  // Struct to hold the rgb color fields
  struct RGB { uint8_t r, g, b; };

  // Pattern of how we want the boundary edge to look (just "paint" one half and we will tile it to both sides of the Sphere
  // the numbers are the numbe of rows the column will modify its head-pointer by.  minus means up (the sphere rows start a 0 on top).  Pos means down.
  static const int8_t waveLUT[COLUMNS/2] = {
    -1,-1,0,0,-1,0,5,6,5,-3,-2,0,0,-1,-1,0,1,0,5,6,5,-3,-4,-3,0,0,-3,-4,-3,0,  1,0,0,0,1,10,11,10,0,-1,-2,-3,-2,-1,0,0,-1,0,5,8,5,1,0,-3,-3,0,-2,-1,-1,0,
  };
  
  // Set up some pre-determined colors to cycle between (these give good contrast between each pair)
  RGB colors[] = {
                  {255,0,0},
                  {0,255,0},
                  {1,65,223},
                  {10,229,0},
                  {218,24,94},
                  {74,160,240},
                  {231,121,67},
                  {72,106,16},
                  {101,16,187},
                  {5,17,137},
                  {196,16,127},
                 };
  static uint8_t bgColorIndex = 0;
  static uint8_t fgColorIndex = 1;
  uint8_t colorArrLen = sizeof(colors) / sizeof(colors[0]);

  // Create instances for the foreground and background color
  static RGB fgColor, bgColor;
  static uint32_t lastCycle = 0;
  uint8_t colorR, colorG, colorB;

  // head just wrapped from 47 → 0, swap rgb fg/bg colors and compute a new fg color
  if (cycle != lastCycle) {    
    bgColor = colors[bgColorIndex];
    fgColor = colors[fgColorIndex];
    bgColorIndex = fgColorIndex;
    fgColorIndex++;
    if(bgColorIndex == colorArrLen - 1) {
       bgColorIndex = 0;
    }
    if(fgColorIndex == colorArrLen - 1 ) {
       fgColorIndex = 0;
    }
    lastCycle = cycle;
  }

  // for each row, figure out which color band it lays in
  for (int row = 0; row < 48; row++) {
    for (int col = 0; col < 120; col++) {

      // Shape the boundary with offsets from our look up table
      int curHead = head + waveLUT[col % COLUMNS/2];
      RGB color = (row <= curHead) ? fgColor : bgColor;

      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = color.r;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = color.g;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = color.b;
    }
  }
}

//#############################################
//  Horizontal bands moving down the Sphere 
//#############################################
void fillBB_hBands() {

  // Create a head pointer which is the leading edge of the first spill color
  uint32_t elapsed = millis();
  uint8_t head = (elapsed * 48) / 3000;      // 0–47 over 3 seconds
  head %= 48;  // Wrap back the the start once head reaches the bottom of the Sphere

  // load with R, G, B for index 0,1,2
  const uint8_t r[3] = {255,0,0};
  const uint8_t g[3] = {0,255,0};
  const uint8_t b[3] = {0,0,255};

  // for each row, figure out which color band it lays in
  for (int row = 0; row < 48; row++) {
    int d = (head - row + 48) % 48;   // distance behind the head this current row is
    uint8_t colorIndex;
    if (d < 8)        colorIndex=0;
    else if (d < 16)   colorIndex=1;
    else if (d < 24)   colorIndex=2;
    else if (d < 32)   colorIndex=0;
    else if (d < 40)   colorIndex=1;
    else if (d < 48)   colorIndex=2;

    // Now write that color all the way around the Sphere
    for (int col = 0; col < 120; col++) {
        uint8_t r8 = 0, g8 = 0, b8 = 0;
        backBuffer[(row * COLUMNS * 3) + (col * 3)]     = r[colorIndex];
        backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g[colorIndex];
        backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b[colorIndex];
    }
  }
}

//#############################################
//  Horizontal stripes with fading color changes
//#############################################
void fillBB_hFade() {

  // Color fade speed
  const float periodMs = 3000.0f;  // one full fade cycle

  // animation speed.  1.0 Hz = one cycle every second
  float omega = 2.0f * PI * 2.0f;     
  //float omega = 2.0f * PI * 0.5f;     

  // graphics are based on the actual time so frames are synced to the clock
  float t = millis();
  float tSec = t * 0.001f;   // seconds
  float animatePhase = omega * tSec;  // Convert to phase to move the image

  // What cycle are we in?
  uint32_t cycle = uint32_t(t / periodMs);

  // Compute the phase angle (in radians) for the given time within the cycle
  float phase = (t - cycle * periodMs) * (2.0f * PI / periodMs);  

  // Brightness: [-1,1] -> [0,255]
  // Use cos so brightness peaks are in phase 
  uint8_t colorBright = (1 - cos(phase)) * 0.5 * 255;

  // Precompute sin of phi to keep it out of the inner loop
  float rowSin[ROWS];
  for (int row = 0; row < ROWS; row++) {
    float phi = ((float(row) / float(ROWS)) * PI) - PI/2.0;
    rowSin[row] = sin(phi * 8 + animatePhase);
  }
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      uint8_t bright = uint8_t((rowSin[row] * 0.5f + 0.5f) * colorBright);
      uint8_t r8 = 0, g8 = 0, b8 = 0;
      switch (cycle % 3) {
        case 0: r8 = bright; break;
        case 1: g8 = bright; break;
        case 2: b8 = bright; break;
      }
      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = r8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
    }
  }
}

//#############################################
//  Vertical stripes with fading color changes
//#############################################
void fillBB_vFade() {

  // Color fade speed
  const float periodMs = 3000.0f;  // one full fade cycle

  // animation speed.  1.0 Hz = one cycle every second
  float omega = 2.0f * PI * 5.0f;     

  // graphics are based on the actual time so frames are synced to the clock
  float t = millis();
  float tSec = t * 0.001f;   // seconds
  float animatePhase = omega * tSec;  // Convert to phase to move the image

  // What cycle are we in?
  uint32_t cycle = uint32_t(t / periodMs);

  // Compute the phase angle (in radians) for the given time within the cycle
  float phase = (t - cycle * periodMs) * (2.0f * PI / periodMs);  

  // Brightness: [-1,1] -> [0,255]
  // Use cos so brightness peaks are in phase 
  uint8_t colorBright = (1 - cos(phase)) * 0.5 * 255;

  // Now map the brightness based on the position
  for (int col = 0; col < COLUMNS; col++) {
    float theta = (col / float(COLUMNS)) * 2*PI;
    for (int row = 0; row < ROWS; row++) {
      float phi = (row / float(ROWS)) * PI - PI/2;

      uint8_t bright = uint8_t((sin(theta * 12 + animatePhase) * 0.5f + 0.5f) * colorBright);

      uint8_t r8 = 0, g8 = 0, b8 = 0;
      switch (cycle % 3) {
        case 0: r8 = bright; break;
        case 1: g8 = bright; break;
        case 2: b8 = bright; break;
      }
      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = r8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
    }
  }
}

//###################################################################
// Fill the backbuffer in preperation for the next displayed frame
// Assumes a 120x48 image imported from Gimp in rgb565 format (two
// bytes per pixel).  Images stored in include/images.h
//###################################################################
void fillBB_image() {

  // Reading an image created by Gimp (loaded from images.h)
  // and expanding/writing the bytes into the framebuffer.  framebuffer always
  // is loaded with rgb888  (one byte per r, g, b) per pixel.
  for (unsigned col = 0; col < imageTable[imageToDisplayIndex]->width; col++) {

    // Start at row 0, this column in the Gimp pixel array
    const uint8_t* p = imageTable[imageToDisplayIndex]->pixel_data + (col * 2);
    for (unsigned row = 0; row < imageTable[imageToDisplayIndex]->height; row++) {
      #if GIMP_RGB565_LITTLE_ENDIAN
        uint16_t rgb565 = (p[1] << 8) | p[0];
      #else
        uint16_t rgb565 = (p[0] << 8) | p[1];
      #endif

      // Read each of the sub-fields to extract rgb from the Gimp 2-byte data
      uint8_t r5 = (rgb565 >> 11) & 0x1F;
      uint8_t g6 = (rgb565 >> 5)  & 0x3F;
      uint8_t b5 =  rgb565        & 0x1F;

      // Expand 5/6-bit channels to full 8-bit range (0–255)
      uint8_t r8 = (r5 << 3) | (r5 >> 2);
      uint8_t g8 = (g6 << 2) | (g6 >> 4);
      uint8_t b8 = (b5 << 3) | (b5 >> 2);

      // Now store the rgb (three bytes per row) data into the backbuffer.  In the rendering (by core-1), 
      // we will add the start bytes, brightness, gamma, and stop bytes before DMA to the dotStars.
      backBuffer[(row * COLUMNS * 3) + (col * 3)] = r8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;

      // Advance to next row, same column
      p += imageTable[imageToDisplayIndex]->width * 2;
    }
  }
}