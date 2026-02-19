
#include <graphicsFunctions.h>


// Each function can be selected by the user from the rotary-encoder/OLED interface
//
// The functions take the absolute time (millis()) and compute animation cycles, phase, 
// animation speed, etc. to compute pixel RGB values anywhere on the sphere.  Then we just
// iterate across the available rows/columns and compute those pixels (basically a coarse
// mapping of the sphere data to the sparse LED matrix overlayed onto it).


// Global definitions

// This is used in the shootingStar animation.  Its the number of stars shooting at any given time
Star stars[NUM_STARS];

// State variables for the fireworks animation.  ROCKET is for the single rising streak, EXPLOSION is when the "spokes" shoot out
typedef enum {
  FIREWORK_ROCKET,
  FIREWORK_EXPLOSION
} FireworkState;

static FireworkState fwState = FIREWORK_ROCKET;
static uint32_t stateStartTime = 0;

// ######################################
// generally helpful utility functions
// ######################################

// linear interpolation.  Return number between a and b.  f is interpolation factor between 0 and 1.0.
float lerp(float a, float b, float f) {
    return a + f * (b - a);
}

// linear interpolation to transition from one color to another.  Pass two RGB color structs, load the "out" struct with the interpolated colors.
// f is the interpolation point (between 0.0 and 1.0)
void lerpColor(RGB& out, RGB& a, RGB& b, float f) {
     out.r = a.r + f * (b.r - a.r);
     out.g = a.g + f * (b.g - a.g);
     out.b = a.b + f * (b.b - a.b);
}

// clamp  For clamping a number between two ranges
float clamp(float val, float minVal, float maxVal) {
  if(val - minVal < .001) {
    return minVal;
  } else if(val - maxVal > .001) {
    return maxVal;
  } else {
    return val;
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
  uint8_t* bbuf = renderer.getBackBuffer();
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
      // Reverse the rows since Gimp stores row 0 at the top and our framebuffer starts with 0 at the bottom.
      bbuf[((ROWS-1-row) * COLUMNS * 3) + (col * 3)] = r8;
      bbuf[((ROWS-1-row) * COLUMNS * 3) + (col * 3 + 1)] = g8;
      bbuf[((ROWS-1-row) * COLUMNS * 3) + (col * 3 + 2)] = b8;

      // Advance to next row, same column
      p += imageTable[imageToDisplayIndex]->width * 2;
    }
  }
}

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
  // Adjust the overall brightness
  r8 = r8Base * bright;
  g8 = g8Base * bright;
  b8 = b8Base * bright;

  // Now write the pixels into the framebuffer
  uint8_t* bbuf = renderer.getBackBuffer();
  for (uint8_t col = 0; col < COLUMNS; col++) {
    for (uint8_t row = 0; row < ROWS; row++) {
      bbuf[(row * COLUMNS * 3) + (col * 3)]     = r8;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
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

  uint32_t head = (elapsed * ROWS) / spillTime_mS;   // What row the spill edge is at
  head %= ROWS;

  // Start the head pointer from the top (row-47) and move down to the bottom (row-0)
  uint32_t  frameBufferHeadPtr = (ROWS-1) - head;
  uint32_t cycle = elapsed / spillTime_mS;        // integer cycle count  

  // Pattern of how we want the boundary edge to look (just "paint" one half and we will tile it to both sides of the Sphere
  // the numbers are the numbe of rows each column will modify its head-pointer by.  This is how we make the edge look "drippy"
  static const int8_t waveLUT[COLUMNS/2] = {
    1,1,0,0,1,0,-5,-6,-5,3,2,0,0,1,1,0,-1,0,-5,-6,-5,3,4,3,0,0,3,4,3,0,  -1,0,0,0,-1,-10,-11,-10,0,1,2,3,2,1,0,0,1,0,-5,-8,-5,-1,0,3,3,0,2,1,1,0,
  };
  
  static int bgColorIndex = 0;
  static int fgColorIndex = 1;
  int colorArrLen = sizeof(contrastColors) / sizeof(contrastColors[0]);

  // Create instances for the foreground and background color
  static RGB fgColor, bgColor;
  static uint32_t lastCycle = 0;

  // head just wrapped from 0 -> 47 swap rgb fg/bg colors and compute a new fg color
  if (cycle != lastCycle) {    
    bgColor = contrastColors[bgColorIndex];
    fgColor = contrastColors[fgColorIndex];
    bgColorIndex = fgColorIndex;
    fgColorIndex++;
    if(bgColorIndex == colorArrLen - 1) {
       bgColorIndex = 0;
    }
    if(fgColorIndex == colorArrLen ) {
       fgColorIndex = 1;
    }
    lastCycle = cycle;
  }

  // For each row, figure out which color band it lays in
  // Start from the top row so it looks like the paint is flowing down
  uint8_t* bbuf = renderer.getBackBuffer();
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < 120; col++) {

      // Shape the boundary with offsets from our look up table
      int curHead = frameBufferHeadPtr + waveLUT[col % COLUMNS/2];
      RGB color = (row <= curHead) ? fgColor : bgColor;

      bbuf[(row * COLUMNS * 3) + (col * 3)]     = color.r;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 1)] = color.g;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 2)] = color.b;
    }
  }
}

//#############################################
//  Horizontal bands moving down the Sphere 
//#############################################
void fillBB_hBands() {

  // Create a head pointer which is the leading edge of the first spill color
  uint32_t elapsed = millis();
  int head = (elapsed * ROWS) / 3000;      // 0–47 over 3 seconds
  head %= ROWS;  // Wrap back the start once head reaches the bottom of the Sphere

  // load with R, G, B and black for index 0,1,2,3
  const uint8_t r[4] = {255,0,0,0};
  const uint8_t g[4] = {0,255,0,0};
  const uint8_t b[4] = {0,0,255,0};

  // for each row, figure out which color band it lays in
  uint8_t* bbuf = renderer.getBackBuffer();
  for (int row = 0; row < ROWS; row++) {
    int d = (head - row + ROWS) % ROWS;   // distance behind the head this current row is
    int colorIndex;

    // Insert a black band between colors
    if (d < 8)        colorIndex=0;
    else if (d < 16)   colorIndex=3;
    else if (d < 24)   colorIndex=1;
    else if (d < 32)   colorIndex=3;
    else if (d < 40)   colorIndex=2;
    else if (d < 48)   colorIndex=3;

    // Now write that color all the way around the Sphere
    for (int col = 0; col < 120; col++) {
        bbuf[(row * COLUMNS * 3) + (col * 3)]     = r[colorIndex];
        bbuf[(row * COLUMNS * 3) + (col * 3 + 1)] = g[colorIndex];
        bbuf[(row * COLUMNS * 3) + (col * 3 + 2)] = b[colorIndex];
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
  int colorBright = (1 - cos(phase)) * 0.5 * 255;

  // Precompute sin of phi to keep it out of the inner loop
  float rowSin[ROWS];
  for (int row = 0; row < ROWS; row++) {
    float phi = ((float(row) / float(ROWS)) * PI) - PI/2.0;
    rowSin[row] = sin(phi * 8 + animatePhase);
  }
  uint8_t* bbuf = renderer.getBackBuffer();
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {

      // smooth sinewave brightness fading
      // just cycle through R/G/B one at a time
      int bright = uint8_t((rowSin[row] * 0.5f + 0.5f) * colorBright);
      uint8_t r8 = 0, g8 = 0, b8 = 0;
      switch (cycle % 3) {
        case 0: r8 = bright; break;
        case 1: g8 = bright; break;
        case 2: b8 = bright; break;
      }
      bbuf[(row * COLUMNS * 3) + (col * 3)]     = r8;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
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
  float omega = 2.0f * PI * 1.0f;     

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
  int colorBright = (1 - cos(phase)) * 0.5 * 255;

  // Now map the brightness based on the position
  uint8_t* bbuf = renderer.getBackBuffer();
  for (int col = 0; col < COLUMNS; col++) {
    float theta = (col / float(COLUMNS)) * 2*PI;
    for (int row = 0; row < ROWS; row++) {
      float phi = (row / float(ROWS)) * PI - PI/2;

      // smooth sinewave brightness fading
      // just cycle through R/G/B one at a time
      int bright = uint8_t((sin(theta * 6 + animatePhase) * 0.5f + 0.5f) * colorBright);

      // just cycle through R/G/B one at a time
      uint8_t r8 = 0, g8 = 0, b8 = 0;
      switch (cycle % 3) {
        case 0: r8 = bright; break;
        case 1: g8 = bright; break;
        case 2: b8 = bright; break;
      }
      bbuf[(row * COLUMNS * 3) + (col * 3)]     = r8;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
    }
  }
}

//#############################################
//  Checker Board with changing colors
//#############################################
void fillBB_checker() {

  // Keep two sets of pointers for the colors befor/after the moving column dividing line
  static int bg0ColorIndex = 0;
  static int fg0ColorIndex = 1;
  static int bg1ColorIndex = 2;
  static int fg1ColorIndex = 3;

  // colors defined in graphicsFunctions.h
  int colorArrLen = sizeof(contrastColors) / sizeof(contrastColors[0]);

  // Use time to control the frequency of color changes
  int colorPeriod = 2000;
  uint32_t elapsed = millis();

  // What cycle are we in?
  uint32_t cycle = elapsed / colorPeriod; 
  static uint32_t lastCycle = 0;

  // The advancing line where the color1 pair replaces color0 pair.  Wrap back to 0 after advancing past COLUMNS
  int head = (elapsed * COLUMNS) / colorPeriod % COLUMNS;  

  // Change colors when head wraps
  if (cycle != lastCycle) {    
    bg0ColorIndex = bg1ColorIndex;
    fg0ColorIndex = fg1ColorIndex;
    bg1ColorIndex+=2;
    fg1ColorIndex+=2;

    if(fg1ColorIndex == colorArrLen + 1 ) {
       bg1ColorIndex = 0;
       fg1ColorIndex = 1;
    }
    lastCycle = cycle;
  }

  // Now map the color based on the position
  uint8_t* bbuf = renderer.getBackBuffer();
  for (int col = 0; col < COLUMNS; col++) {
    int d = (head - col + COLUMNS) % COLUMNS;   // distance behind the head this current col is
    int fgIndex, bgIndex;

    // Pick the color pair based on where we are relative to the moving head dividing line
    if (d > 0 && d <= head) {
      fgIndex = fg1ColorIndex;
      bgIndex = bg1ColorIndex;
    } else {  
      fgIndex = fg0ColorIndex;
      bgIndex = bg0ColorIndex;
    }
    for (int row = 0; row < ROWS; row++) {

      // shift by 3 so we "coarseify" the xor to happen across eight row/col bands
      bool fg = ((row >> 3) ^ (col >> 3)) & 1;
      RGB color;
      if(fg) {
        color = contrastColors[fgIndex];
      } else { 
        color = contrastColors[bgIndex];
      }
      bbuf[(row * COLUMNS * 3) + (col * 3)]     = color.r;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 1)] = color.g;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 2)] = color.b;
    }
  }
}

//####################################
//  Whole globe sized Pacman chomping
//####################################
void fillBB_pacman() {

  // need the backbuffer pointer to write to
  uint8_t* bbuf = renderer.getBackBuffer();

  // How quickly we do framebuffer updates (in ms)
  uint16_t animatePeriod = 500;

  // Get the colors
  static palette c;

  // Use time to control the animation
  uint32_t elapsed = millis();

  // What cycle are we in?
  uint32_t cycle = elapsed / animatePeriod; 
  static uint32_t lastCycle = 0;

  // Draw the yellow ball
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      gPrim.writePixel(bbuf,col, row, c.yellow);
    }
  }

  // Draw the left eye (a circle with a notch)
  gPrim.drawCircle(bbuf,30,32,4,c.black);
  gPrim.drawTriangle(bbuf,{28,32}, {20,42}, {20,22}, c.yellow);

  // Draw the right eye
  gPrim.drawCircle(bbuf,50,32,4,c.black);
  gPrim.drawTriangle(bbuf,{52,32}, {60,42}, {60,22}, c.yellow);

  // Animate the mouth by drawing open/close based on the cycle we are in
  static bool open = false;
  if(cycle != lastCycle) {
    open = !open;
    lastCycle = cycle;
  }

  if(open) {
    // open mouth is an ellipse.  radiusX is the width, radiusY is the height
    gPrim.drawEllipse(bbuf,40, 19, 25, 8, 0, c.black);
  } else {

    // Draw the closed mouth (simple horizontal line)
    gPrim.drawRect(bbuf,15, 19, 65, 19, 0, c.black);
  }
}

//#########################
//  Pacman chasing ghosts
//#########################
void fillBB_pacman1() {

  // need the backbuffer pointer to write to
  uint8_t* bbuf = renderer.getBackBuffer();

  // How quickly we do framebuffer updates (in ms)
  uint16_t animatePeriod = 500;

  // Get the colors
  static palette c;

  // Use time to control the animation
  uint32_t elapsed = millis();

  // What cycle are we in?
  uint32_t cycle = elapsed / animatePeriod; 
  static uint32_t lastCycle = 0;

  // Fill the black background
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      gPrim.writePixel(bbuf, col, row, c.black);
    }
  }
  // Animate the mouth by drawing open/close based on the cycle we are in
  static bool mouthOpen = false;
  static bool jumpUp = false;
  if(cycle != lastCycle) {
    mouthOpen = !mouthOpen;
    jumpUp = !jumpUp;
    lastCycle = cycle;
  }
  // Draw Pacman
  gComp.drawPacman(bbuf,41,24,c.yellow,c.black,mouthOpen);

  // Draw the ghosts
  gComp.drawGhost(bbuf,65, 25, c.red, c.black, c.white,jumpUp);
  gComp.drawGhost(bbuf,85, 25, c.blue, c.black, c.white,!jumpUp);
  gComp.drawGhost(bbuf,105, 25, c.orange, c.black, c.white,jumpUp);

  // Draw the Pacman food dots
  gPrim.drawCircle(bbuf,5, 23, 2, c.yellow);
  gPrim.drawCircle(bbuf,15, 23, 2, c.yellow);
  gPrim.drawCircle(bbuf,25, 23, 2, c.yellow);
}


//#########################
//  Expanding Diamonds
//#########################
void fillBB_diamond() {

  // need the backbuffer pointer to write to
  uint8_t* bbuf = renderer.getBackBuffer();

  // How quickly we do framebuffer updates (in ms)
  uint16_t animatePeriod = 50;
  constexpr int borderWidth = 5;
  static int animateCount = borderWidth;

  // Get the colors
  static palette c;

  // Use time to control the animation
  uint32_t elapsed = millis();

  // What cycle are we in?
  uint32_t cycle = elapsed / animatePeriod; 
  static uint32_t lastCycle = 0;

  // Fill the background
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      gPrim.writePixel(bbuf,col, row, c.black);
    }
  }

  // Increment the animate count when we see a cycle transition
  static int animateIncrement = 1;
  if( cycle != lastCycle){
    lastCycle = cycle;
    if(animateCount >= 19) {
      animateIncrement = -1;
    }
    if(animateCount <= borderWidth) {
      animateIncrement = 1;
    }
    animateCount += animateIncrement;
  }

  // diamonds grow and shrink
  gPrim.drawDiamond(bbuf,20, 24, animateCount, animateCount, c.yellow);
  gPrim.drawDiamond(bbuf,20, 24, animateCount-borderWidth, animateCount-borderWidth, c.purple);

  gPrim.drawDiamond(bbuf,60, 24, animateCount, animateCount, c.blue);
  gPrim.drawDiamond(bbuf,60, 24, animateCount-borderWidth, animateCount-borderWidth, c.yellow);

  gPrim.drawDiamond(bbuf,100, 24, animateCount, animateCount, c.purple);
  gPrim.drawDiamond(bbuf,100, 24, animateCount-borderWidth, animateCount-borderWidth, c.yellow);
}

//#########################
//  Expanding Flower
//#########################
void fillBB_flower() {

  // need the backbuffer pointer to write to
  uint8_t* bbuf = renderer.getBackBuffer();

  // How quickly we do framebuffer updates (in ms)
  uint16_t animatePeriod = 50;
  static int animateCount = 1;

  // Get the colors
  static palette c;

  // Use time to control the animation
  uint32_t elapsed = millis();

  // What cycle are we in?
  uint32_t cycle = elapsed / animatePeriod; 
  static uint32_t lastCycle = 0;

  // Fill the background
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      gPrim.writePixel(bbuf,col, row, c.black);
    }
  }

  // Increment the animate count when we see a cycle transition
  static int animateIncrement = 1;
  if( cycle != lastCycle){
    lastCycle = cycle;
    if(animateCount >= ROWS-1) {
      animateIncrement = -1;
    }
    if(animateCount <= 1) {
      animateIncrement = 1;
    }
    animateCount += animateIncrement;
  }

  // From top to bottom
  // Just pull the middle vertex up/down to grow the triangles
  gPrim.drawTriangle(bbuf,{0,0},{20,animateCount},{39,0}, c.red);
  gPrim.drawTriangle(bbuf,{40,0},{60,animateCount},{79,0}, c.blue);
  gPrim.drawTriangle(bbuf,{80,0},{100,animateCount},{119,0}, c.green);

  // From bottom to top
  gPrim.drawTriangle(bbuf,{0,47},{0,47-animateCount},{19,47}, c.blue);
  gPrim.drawTriangle(bbuf,{20,47},{40,47-animateCount},{59,47}, c.green);
  gPrim.drawTriangle(bbuf,{60,47},{80,47-animateCount},{99,47}, c.red);
  gPrim.drawTriangle(bbuf,{100,47},{119,47-animateCount},{119,47}, c.blue);

}

//#######################################
//  Main spiral draw engine.  
//#######################################
void cycle_spiral(int numSpirals, int thickness, int twist){

  // need the backbuffer pointer to write to
  uint8_t* bbuf = renderer.getBackBuffer();

  // How quickly we do framebuffer updates (in ms)
  uint16_t animatePeriod = 50;
  static int animateCount = 1;
  static int fg0ColorIndex = 0;
  static int fg1ColorIndex = 1;
  RGB bgColor = {0,0,0};
  int colorArrLen = sizeof(spiralColors) / sizeof(spiralColors[0]);

  static uint32_t phase = 0;
  static uint32_t lastColorSwitchTime = 0;
  uint32_t now = millis();

  // What animate cycle are we in?
  uint32_t cycle = now / animatePeriod; 
  static uint32_t lastCycle = 0;

  // Increment the animate count when we see a cycle transition
  // We increment up and down so the pattern rises/falls
  // Switch colors once the spiral rise and falls once
  static int animateIncrement = 1;
  static bool switchColors = false;
  if( cycle != lastCycle){
    lastCycle = cycle;
    if(animateCount >= ROWS-1) {
      animateIncrement = -1;
    }
    if(animateCount <= 1) {
      animateIncrement = 1;
      switchColors = true;
    }
    animateCount += animateIncrement;
  }

  // Change color once the spiral has done one rise/fall
  if (switchColors) {    
    switchColors = false;
    fg0ColorIndex = fg1ColorIndex;
    fg1ColorIndex += 1;

    if(fg1ColorIndex == colorArrLen) {
       fg1ColorIndex = 0;
    }
  }

  // Fill the background
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      gPrim.writePixel(bbuf,col, row, bgColor);
    }
  }
  gPrim.drawSpiral(bbuf,phase, -twist, numSpirals, thickness, animateCount, spiralColors[fg0ColorIndex]);
}

//#######################################
//  Right twist Spiral around the Sphere
//#######################################
void fillBB_spiralR(){

  // Set how many spirals and there size
  constexpr int numSpirals = 6;
  constexpr int thickness = 4;  // How many pixels wide are the spirals
  constexpr int K = -1;  // Spiral twist factor

  cycle_spiral(numSpirals, thickness, K);

}

//#######################################
//  Left twist Spiral around the Sphere
//#######################################
void fillBB_spiralL(){

  // Set how many spirals and there size
  constexpr int numSpirals = 8;
  constexpr int thickness = 8;  // How many pixels wide are the spirals
  constexpr int K = 1;  // Spiral twist factor

  cycle_spiral(numSpirals, thickness, K);
}

//##################################
//  Double Spiral around the Sphere
//##################################
void fillBB_spiralD(){

  uint8_t* bbuf = renderer.getBackBuffer();

  constexpr int K = 1;  // Spiral twist factor
  constexpr int numSpirals = 6;
  constexpr int thickness = 3;  // How many pixels wide are the spirals
  static int fg0ColorIndex = 0;
  static int fg1ColorIndex = 1;

  RGB bgColor = {0,0,0};
  int colorArrLen = sizeof(spiralColors) / sizeof(spiralColors[0]);

  // color change period
  constexpr uint32_t spiralRevPeriod = 3000; //in mS 

  // How quickly we do framebuffer updates (in ms)
  uint16_t animatePeriod = 50;
  static int animateCount = 1;

  static uint32_t phase = 0;
  static uint32_t lastColorSwitchTime = 0;
  uint32_t now = millis();

  // What animate cycle are we in?
  uint32_t cycle = now / animatePeriod; 
  static uint32_t lastCycle = 0;

  // Increment the animate count when we see a cycle transition
  // We increment up and down so the pattern rises/falls
  // Switch colors once the spiral rise and falls once
  static int animateIncrement = 1;
  static bool switchColors = false;
  if( cycle != lastCycle){
    lastCycle = cycle;
    if(animateCount >= ROWS-1) {
      animateIncrement = -1;
    }
    if(animateCount <= 1) {
      animateIncrement = 1;
      switchColors = true;
    }
    animateCount += animateIncrement;
  }

  // Change color once the spiral has done one rise/fall
  if (switchColors) {    
    switchColors = false;
    lastColorSwitchTime = now;
    fg0ColorIndex = fg1ColorIndex;
    fg1ColorIndex += 1;

    if(fg1ColorIndex == colorArrLen) {
       fg1ColorIndex = 0;
    }
  }


  // Fill the background
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      gPrim.writePixel(bbuf,col, row, bgColor);
    }
  }
  // One spiral is fixed height, the other will grow from the bottom to the top
  gPrim.drawSpiral(bbuf, phase, K, numSpirals, thickness, animateCount, spiralColors[fg0ColorIndex]);
  gPrim.drawSpiral(bbuf, phase, -K, numSpirals, thickness, animateCount, spiralColors[fg1ColorIndex]);

}

//#############################
//  Animated blinking eyeballs
//#############################
void fillBB_eyeball() {

  uint8_t* bbuf = renderer.getBackBuffer();

  // How quickly we do framebuffer updates (in ms)
  constexpr uint16_t animatePeriod = 1000;
  constexpr uint16_t blinkPeriod = 5;  // how fast to blink
  constexpr int ANIMATE_CYCLES_UNTIL_TRIGGER = 1;  // how often to blink
  static int blinkTrigger = ANIMATE_CYCLES_UNTIL_TRIGGER;  
  static int blinkToRow = ROWS - 1;  // How far down the blink is
  static int whichEyeToBlink = 0;

  // Get the colors
  static palette c;

  // Use time to control the animation
  uint32_t elapsed = millis();

  // What cycle are we in?
  uint32_t cycle = elapsed / animatePeriod; 
  uint32_t blinkCycle = elapsed / blinkPeriod; 
  static uint32_t lastCycle = 0;
  static uint32_t lastBlinkCycle = 0;

  // Fill the black background
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      gPrim.writePixel(bbuf,col, row, c.black);
    }
  }
  // Animate the eyeball by offsetting the iris/pupil based on the cycle we are in
  static EYEBALL_MOVE position[5] = {CENTER, LEFT, RIGHT, UP, DOWN};
  static int posIndex = 0;
  if(cycle != lastCycle) {
    posIndex = int(random(4));
    lastCycle = cycle;
    blinkTrigger--;  // count down until time to blink
    if(blinkTrigger < 0) {
      lastBlinkCycle = blinkCycle;
    }
  }

  // What to do when the blink trigger is active
  if(blinkTrigger < 0) {
    if(blinkCycle != lastBlinkCycle) {
      lastBlinkCycle = blinkCycle;
      blinkToRow-=4;
      if(blinkToRow <= 4) {
        whichEyeToBlink = int(random(3));
        blinkTrigger = ANIMATE_CYCLES_UNTIL_TRIGGER; 
        blinkToRow = ROWS - 1;
      }
    }
  }
  // Draw Eyeball
  gComp.drawEyeball(bbuf,20, 25, 14, c.mediumblue, c.black, c.mediumgray, position[posIndex] );
  gComp.drawEyeball(bbuf,60, 25, 14, c.olivegreen, c.black, c.mediumgray, position[posIndex] );
  gComp.drawEyeball(bbuf,100, 25, 14, c.brown, c.black, c.mediumgray, position[posIndex] );

  if(blinkTrigger < 0) {
    // Mask the eyeball from the top down over the blinkPeriod 
    if(whichEyeToBlink == 0) {
      gPrim.drawRect(bbuf,6,blinkToRow,34,47, 0,c.black);
    } else if(whichEyeToBlink == 1) {
      gPrim.drawRect(bbuf,46,blinkToRow,74,47, 0,c.black);
    } else {
      gPrim.drawRect(bbuf,86,blinkToRow,114,47, 0,c.black);
    }
  }
}

//###############################################################################
//############# Helper functions for the Pinecrest animation ###################
//###############################################################################

//######## rendering the owl blink #############
bool renderBlink(uint32_t now) {
  if (!owl.blinkActive) return false;

  uint32_t elapsed = now - owl.blinkStartTime;
  if (elapsed >= BLINK_DURATION_MS) {
    owl.blinkActive = false;
    return false;
  }
  return true;  // eyes closed (or partially, if you interpolate)
}

//#########  Edge based state update based on what time we have reached
void updateOwlEvents(uint32_t now, uint32_t sceneStartTime, const OwlEvent* timeline, size_t eventCount, size_t& owlNextEvent) {

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

//########## rendering for each of the marshmallow phases #############
// Just sit there on the stick
void renderRaw(uint32_t t, palette& colors) {
    mm.rotation = 0;
    mm.cy       = 26;
    mm.halfSize = 3;
    mm.color    = colors.white;
}

// Strart toasting - turn brown
void renderToasting(uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.rotation = lerp(0,10,u);
    mm.cy       = 26;
    mm.halfSize = 3;
    lerpColor(mm.color,colors.white, colors.lightbrown, u);
}

// Strart melting/dropping
void renderMelting(uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.rotation = lerp(10,20,u);
    //mm.cy = lerp(26,24,u);
    mm.cy       = 26;
    mm.halfSize = 3;
    lerpColor(mm.color, colors.lightbrown, colors.brown, u);
}

// Drop into the file
void renderDropping(uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.cy = lerp(26,18,u);
    mm.rotation = lerp(20,25,u);
    mm.halfSize = 3;
    lerpColor(mm.color, colors.brown, colors.darkgray, u);
}
// Burned sitting in the fire
void renderBurnt(uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.cy = lerp(18,10,u);
    mm.rotation = lerp(25,29,u);
    mm.halfSize = 3;
    lerpColor(mm.color, colors.darkgray, colors.black, u);
}

// Up in smoke
void renderSmoke(uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.cy = lerp(10,44,u);
    //lerpColor(mm.color, colors.white, colors.black, u);
    mm.color = colors.white;
}

//#########  Edge triggered transition state change based on what time we reach
void updateMarshmallowEvents(uint32_t now, uint32_t sceneStartTime, const MarshmallowEvent* mmTimeline, size_t eventCount, size_t& mmNextEvent, palette& colors) {

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

// Call the rendering functions based on what state we are in
void renderMarshmallow(uint32_t now, palette& colors) {
  uint8_t* bbuf = renderer.getBackBuffer();
  uint32_t phaseElapsed = now - mm.phaseStartTime;

    switch (mm.phase) {
        case MM_RAW:
            renderRaw(phaseElapsed, colors);
            break;

        case MM_TOASTING:
            renderToasting(phaseElapsed,colors);
            break;

        case MM_MELTING:
            renderMelting(phaseElapsed,colors);
            break;

        case MM_DROPPING:
            renderDropping(phaseElapsed,colors);
            break;

        case MM_BURNT:
            renderBurnt(phaseElapsed,colors);
            break;

        case MM_SMOKE:
            renderSmoke(phaseElapsed,colors);
            break;
    }
    if(mm.phase == MM_SMOKE) {
      static int arcDepth = -2;
      
      // Once the smoke gets to the top, turn it off
      if(mm.cy >= 44) {
        mm.color = colors.black ;
      }
      gPrim.drawArc(bbuf,{mm.cx,mm.cy-3},{mm.cx-arcDepth,mm.cy},{mm.cx,mm.cy+3},1,mm.color);
      arcDepth = arcDepth * -1;  // Alternate the arc back and forth
    } else {
      gPrim.drawQuad(bbuf,mm.cx-mm.halfSize, mm.cy+mm.halfSize, mm.cx-mm.halfSize, mm.cy-mm.halfSize, mm.cx+mm.halfSize, mm.cy-mm.halfSize, mm.cx+mm.halfSize, mm.cy+mm.halfSize, mm.rotation, mm.color);
    }
}

  
//#############################
//  Animated Pincrest Scene
//#############################
void fillBB_pinecrest() {

  uint8_t* bbuf = renderer.getBackBuffer();

  // Get the colors
  static palette c;
  RGB flames[] = {c.red,c.yellow};

  static uint32_t sceneStartTime = 0;
  static bool timelineInitialized = false;

  // Use time to control the animation
  uint32_t now = millis();

  // Start the scene time
  if (!timelineInitialized) {
    sceneStartTime = now;
    timelineInitialized = true;
  }

  // Fill the background
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      gPrim.writePixel(bbuf,col, row, c.black);
    }
  }

  // campfire
  gPrim.drawTriangle(bbuf,{50,7},{55,random(16,21)},{60,7},flames[random(2)]);
  gPrim.drawTriangle(bbuf,{60,7},{65,random(16,21)},{70,7},flames[random(2)]);
  gPrim.drawTriangle(bbuf,{70,7},{75,random(16,21)},{80,7},flames[random(2)]);
  gPrim.drawTriangle(bbuf,{55,7},{60,random(16,21)},{65,7},flames[random(2)]);  
  gPrim.drawTriangle(bbuf,{65,7},{70,random(16,21)},{75,7},flames[random(2)]);


  // roasting stick
  gPrim.drawLine(bbuf,71,25,90,18,1,c.gray);  

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
  renderMarshmallow(now, c);

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
  bool blink  = renderBlink(now);
  bool squawk = owl.squawking;
  gComp.drawOwl(bbuf,100, 25, blink, squawk);

  // PINECREST letters
  font_7x7 f;
  gPrim.drawLetter(bbuf,f.P,0 ,32, c.black, c.green);
  gPrim.drawLetter(bbuf,f.I,7 ,32, c.black, c.green);
  gPrim.drawLetter(bbuf,f.N,14 ,32, c.black, c.green);
  gPrim.drawLetter(bbuf,f.E,21 ,32, c.black, c.green);
  gPrim.drawLetter(bbuf,f.C,28 ,32, c.black, c.green);
  gPrim.drawLetter(bbuf,f.R,35 ,32, c.black, c.green);
  gPrim.drawLetter(bbuf,f.E,42 ,32, c.black, c.green);
  gPrim.drawLetter(bbuf,f.S,49 ,32, c.black, c.green);
  gPrim.drawLetter(bbuf,f.T,56 ,32, c.black, c.green);

  gPrim.drawLetter(bbuf,f.N2,10 ,18, c.black, c.green);
  gPrim.drawLetter(bbuf,f.N0,17 ,18, c.black, c.green);
  gPrim.drawLetter(bbuf,f.N2,24 ,18, c.black, c.green);
  gPrim.drawLetter(bbuf,f.N6,31 ,18, c.black, c.green);

  // Reset the scene
  if(now > sceneStartTime + 11500) {
    timelineInitialized = false;
    owlNextEvent = 0;
    mmNextEvent = 0;
  }
}


//###############################################################################
//  Animated Shooting Star
//  We have an active "star" (head) that traces a path into the framebuffer, then
//  we use a fade function to go erase the trails with a defined time-constant.
//  Renders a a bright star shooting across the sphere with the tail fading off
//  behind it.
//###############################################################################
//     Helper functions 
void fadeFramebuffer(uint8_t decay) {
  uint8_t* bbuf = renderer.getBackBuffer();
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      for (int c = 0; c < 3; c++) {
        int idx = ((row * COLUMNS + col) * 3) + c;
        uint8_t v = bbuf[idx];
        bbuf[idx] = (v > decay) ? (v - decay) : 0;
      }
    }
  }
}

// Write the leading pixel for the shootingStar
void writeHead(int col, int row, RGB color) {
  uint8_t* bbuf = renderer.getBackBuffer();
  int idx = (row * COLUMNS + col) * 3;
  bbuf[idx]     = color.r;
  bbuf[idx + 1] = color.g;
  bbuf[idx + 2] = color.b;
}

// Set up parameters for multiple shootingStars
void initShootingStars() {
  for (int i = 0; i < NUM_STARS; i++) {
    stars[i].x  = random(0, COLUMNS);
    stars[i].y  = random(0, ROWS);
    stars[i].vx = random(-30, 30) / 100.0;   // subtle drift
    stars[i].vy = - (random(80, 140) / 100.0);

    stars[i].r = random(0, 255);
    stars[i].g = random(0, 255);
    stars[i].b = random(0, 255);
  }
}

// Main rendering function
void fillBB_shootingStar() {

  // Iterate over the entire framebuffer subtracting brightness from each
  // pixel until we fade to black
  fadeFramebuffer(10);   // larger number fades quicker

  uint8_t* bbuf = renderer.getBackBuffer();
  for (int i = 0; i < NUM_STARS; i++) {

    Star *s = &stars[i];

    s->x += s->vx;
    s->y += s->vy;

    // wrap columns
    if (s->x < 0)        s->x += COLUMNS;
    if (s->x >= COLUMNS) s->x -= COLUMNS;

    // respawn
    if (s->y < 0) {
      s->x = random(0, COLUMNS);
      s->y = ROWS - 1;

      // Alter these to change the glide slope
      //s->vx = random(-30, 30) / 100.0;  // More vertical orientation
      s->vx = random(-300, 300) / 100.0;  // More horizontal motion
      s->vy = - (random(80, 140) / 100.0);

      // Every now and then throw in a "flare"
      if (random(0, 100) < 3) {
        s->r = s->g = s->b = 255;
      } else {
        s->r = random(0, 255);
        s->g = random(0, 255);
        s->b = random(0, 255);
      }
    }

    int col = (int)s->x;
    int row = (int)s->y;

    int idx = (row * COLUMNS + col) * 3;
    bbuf[idx]     = s->r;
    bbuf[idx + 1] = s->g;
    bbuf[idx + 2] = s->b;
  }
}

// ###############################################
// Functions for the fireworks animation
// A classic rocket streaking up and exploding into 
// a starburst which then fades out
// ###############################################
uint8_t NUM_EXPLOSION_STARS;
void initRocket() {

  Star *s = &stars[0];

  s->x  = random(0, COLUMNS);
  s->y  = 0;   // bottom

  // diagonal upward motion
  s->vx = random(-100, 100) / 100.0;   // add horizontal drift
  s->vy = random(120, 160) / 100.0;  // strong upward velocity
  s->r = 255;
  s->g = 200;
  s->b = 100;

  fwState = FIREWORK_ROCKET;
  stateStartTime = millis();
}

void initExplosion(float x, float y) {

  uint8_t r,g,b;
  r = random(150, 255);
  g = random(100, 255);
  b = random(100, 255);
  float speed = random(60, 160) / 100.0;
  NUM_EXPLOSION_STARS =  random(16,20);

  for (int i = 0; i < NUM_EXPLOSION_STARS; i++) {
    Star *s = &stars[i];

    s->x = x;
    s->y = y;

    float angle = (TWO_PI / NUM_EXPLOSION_STARS) * i;

    s->vx = cos(angle) * speed;
    s->vy = sin(angle) * speed;
    s->r = r;
    s->g = g;
    s->b = b;
    s->age = 0;
    //s->maxAge = random(18, 35);   // ~0.3–0.6 sec at 60fps
    s->maxAge = 15;            // Keep them all the same size  
    s->active = true;
  }

  fwState = FIREWORK_EXPLOSION;
  stateStartTime = millis();
}

// ###########################################################
// Main Fireworks code
// Control the rocket/explosion phases with state variables
// ###########################################################
void fillBB_fireworks() {

  switch (fwState) {
    case FIREWORK_ROCKET: {
      fadeFramebuffer(30);   // fade the rocket trail faster
      Star *s = &stars[0];
      s->x += s->vx;
      s->y += s->vy;

      // wrap horizontally across the column seam
      if (s->x < 0)        s->x += COLUMNS;
      if (s->x >= COLUMNS) s->x -= COLUMNS;

      int col = (int)s->x;
      int row = (int)s->y;

      // Write the bright head of the rocket path
      if (row >= 0 && row < ROWS) {
        writeHead(col, row, {s->r, s->g, s->b});
      }

      // explode at 2/3 height.
      if (s->y > (ROWS * 0.66)) {
        initExplosion(s->x, s->y);
      }

    } break;

    case FIREWORK_EXPLOSION: {
      fadeFramebuffer(8);   // Slower fade for the firework burst
      for (int i = 0; i < NUM_EXPLOSION_STARS; i++) {
        Star *s = &stars[i];

        if (!s->active) continue;

        s->x += s->vx;
        s->y += s->vy;

        s->vx *= 0.97;   // optional drag
        s->vy *= 0.97;

       if (s->x < 0)        s->x += COLUMNS;
       if (s->x >= COLUMNS) s->x -= COLUMNS;

        s->age++;

        if (s->age >= s->maxAge) {
          s->active = false;
          continue;
        }

        int col = (int)s->x;
        int row = (int)s->y;
        
        if (col >= 0 && col < COLUMNS && row >= 0 && row < ROWS) {
          writeHead(col, row, {s->r, s->g, s->b});
        }
      }

      // after 700ms restart cycle
      if (millis() - stateStartTime > 700) {
        initRocket();
      }
    } break;
  }
}


//##########################################
// Helper functions for the sparkShower
// An animation the floods "sparks" out the
// top of the sphere to cascade down until 
// fading out
//##########################################
void shiftDown() {
  uint8_t* bbuf = renderer.getBackBuffer();
  for (int row = 0; row < ROWS - 1; row++) {
    for (int col = 0; col < COLUMNS; col++) {
      int dst = (row * COLUMNS + col) * 3;
      int src = ((row + 1) * COLUMNS + col) * 3;
      bbuf[dst]     = bbuf[src];
      bbuf[dst + 1] = bbuf[src + 1];
      bbuf[dst + 2] = bbuf[src + 2];
    }
  }

  // Clear the top row (we'll re-inject)
  int topRow = ROWS - 1;
  for (int col = 0; col < COLUMNS; col++) {
    int idx = (topRow * COLUMNS + col) * 3;
    bbuf[idx]     = 0;
    bbuf[idx + 1] = 0;
    bbuf[idx + 2] = 0;
  }
}

// Main spark rendering function
void fillBB_sparkShower() {

  fadeFramebuffer(7);   // short persistence
  shiftDown();          // actual downward motion

  int topRow = ROWS - 1;

  uint8_t* bbuf = renderer.getBackBuffer();
  for (int col = 0; col < COLUMNS; col++) {

    // emission probability per column
    if (random(0, 1000) < 80) {
      int idx = (topRow * COLUMNS + col) * 3;
      uint8_t v = random(150, 255);

      bbuf[idx]     = random(255);
      bbuf[idx + 1] = random(255);
      bbuf[idx + 2] = random(255);
    }
  }
}