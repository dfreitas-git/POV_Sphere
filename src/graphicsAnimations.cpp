
#include <graphicsParticles.h>
#include <graphicsPrimitives.h>
#include <graphicsComposites.h>
#include <graphicsAnimations.h>
#include <graphicsScenes.h>
#include <graphicsGlobals.h>
#include <renderer.h>

extern Renderer renderer;
extern GraphicsParticles gParticles;

void fadeWrapper(FrameBuffer bbuf) {
  gAnim.fade(bbuf);
}
void hFadeWrapper(FrameBuffer bbuf) {
  gAnim.hFade(bbuf);
}
void vFadeWrapper(FrameBuffer bbuf) {
  gAnim.vFade(bbuf);
}
void hBandsWrapper(FrameBuffer bbuf) {
  gAnim.hBands(bbuf);
}
void paintWrapper(FrameBuffer bbuf) {
  gAnim.paint(bbuf);
}
void checkerWrapper(FrameBuffer bbuf) {
  gAnim.checker(bbuf);
}
void spiralRWrapper(FrameBuffer bbuf) {
  gAnim.spiralR(bbuf);
}
void spiralLWrapper(FrameBuffer bbuf) {
  gAnim.spiralL(bbuf);
}
void spiralDWrapper(FrameBuffer bbuf) {
  gAnim.spiralD(bbuf);
}
void diamondWrapper(FrameBuffer bbuf) {
  gAnim.diamond(bbuf);
}
void flowerWrapper(FrameBuffer bbuf) {
  gAnim.flower(bbuf);
}
void eyeballWrapper(FrameBuffer bbuf) {
  gAnim.eyeball(bbuf);
}
void pacmanWrapper(FrameBuffer bbuf) {
  gAnim.pacman(bbuf);
}
void pacman1Wrapper(FrameBuffer bbuf) {
  gAnim.pacman1(bbuf);
}
void shootingStarWrapper(FrameBuffer bbuf) {
  gAnim.shootingStar(bbuf);
}
void sparkShowerWrapper(FrameBuffer bbuf) {
  gAnim.sparkShower(bbuf);
}

// These call functions in Scenes class.  Just leaving the wrappers here to keep them all grouped together
void pinecrestWrapper(FrameBuffer bbuf) {
  gScene.pinecrest(bbuf);
}
void fireworksWrapper(FrameBuffer bbuf) {
  gScene.fireworks(bbuf);
}


// ###########################
// Helpful utility functions
// ############################

// linear interpolation.  Return number between a and b.  f is interpolation factor between 0 and 1.0.
float GraphicsAnimations::lerp(float a, float b, float f) {
    return a + f * (b - a);
}

// linear interpolation to transition from one color to another.  Pass two RGB color structs, load the "out" struct with the interpolated colors.
// f is the interpolation point (between 0.0 and 1.0)
void GraphicsAnimations::lerpColor(RGB& out, RGB& a, RGB& b, float f) {
     out.r = a.r + f * (b.r - a.r);
     out.g = a.g + f * (b.g - a.g);
     out.b = a.b + f * (b.b - a.b);
}

// clamp  For clamping a number between two ranges
float GraphicsAnimations::clamp(float val, float minVal, float maxVal) {
  if(val - minVal < .001) {
    return minVal;
  } else if(val - maxVal > .001) {
    return maxVal;
  } else {
    return val;
  }
}

// Fill the sphere background
void GraphicsAnimations::clearBackground(FrameBuffer bbuf) {
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      gPrim.writePixel(bbuf,col, row, {0,0,0});
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
void GraphicsAnimations::fade(FrameBuffer bbuf) {

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
  for (uint8_t col = 0; col < COLUMNS; col++) {
    for (uint8_t row = 0; row < ROWS; row++) {
      bbuf[(row * COLUMNS * 3) + (col * 3)]     = r8;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 1)] = g8;
      bbuf[(row * COLUMNS * 3) + (col * 3 + 2)] = b8;
    }
  }
}

//#############################################
//  Horizontal stripes with fading color changes
//#############################################
void GraphicsAnimations::hFade(FrameBuffer bbuf) {

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
void GraphicsAnimations::vFade(FrameBuffer bbuf) {

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
//  Horizontal bands moving down the Sphere 
//#############################################
void GraphicsAnimations::hBands(FrameBuffer bbuf) {

  // Create a head pointer which is the leading edge of the first spill color
  uint32_t elapsed = millis();
  int head = (elapsed * ROWS) / 3000;      // 0–47 over 3 seconds
  head %= ROWS;  // Wrap back the start once head reaches the bottom of the Sphere

  // load with R, G, B and black for index 0,1,2,3
  const uint8_t r[4] = {255,0,0,0};
  const uint8_t g[4] = {0,255,0,0};
  const uint8_t b[4] = {0,0,255,0};

  // for each row, figure out which color band it lays in
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

//###############################################################
//  Color flowing out the top, down the Sphere in dripping sheets
//  like paint poured on top of the Sphere.
//###############################################################
void GraphicsAnimations::paint(FrameBuffer bbuf) {

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
//  Checker Board with changing colors
//#############################################
void GraphicsAnimations::checker(FrameBuffer bbuf) {

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

//#######################################
//  Main spiral draw engine.  
//#######################################
void GraphicsAnimations::cycle_spiral(FrameBuffer bbuf, int numSpirals, int thickness, int twist){

  // How quickly we do framebuffer updates (in ms).  Can't be faster than 16ms which is the graphics task loop time.
  uint16_t animatePeriod = 15;
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
void GraphicsAnimations::spiralR(FrameBuffer bbuf) {

  // Set how many spirals and there size
  constexpr int numSpirals = 6;
  constexpr int thickness = 4;  // How many pixels wide are the spirals
  constexpr int K = -1;  // Spiral twist factor

  cycle_spiral(bbuf, numSpirals, thickness, K);

}

//#######################################
//  Left twist Spiral around the Sphere
//#######################################
void GraphicsAnimations::spiralL(FrameBuffer bbuf) {

  // Set how many spirals and there size
  constexpr int numSpirals = 8;
  constexpr int thickness = 8;  // How many pixels wide are the spirals
  constexpr int K = 1;  // Spiral twist factor

  cycle_spiral(bbuf, numSpirals, thickness, K);
}

//##################################
//  Double Spiral around the Sphere
//##################################
void GraphicsAnimations::spiralD(FrameBuffer bbuf) {

  constexpr int K = 1;  // Spiral twist factor
  constexpr int numSpirals = 6;
  constexpr int thickness = 3;  // How many pixels wide are the spirals
  static int fg0ColorIndex = 0;
  static int fg1ColorIndex = 1;

  RGB bgColor = {0,0,0};
  int colorArrLen = sizeof(spiralColors) / sizeof(spiralColors[0]);

  // color change period
  constexpr uint32_t spiralRevPeriod = 3000; //in mS 

  // How quickly we do framebuffer updates (in ms).  Can't be faster than 16ms which is the graphics task loop time.
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

//#########################
//  Expanding Diamonds
//#########################
void GraphicsAnimations::diamond(FrameBuffer bbuf) {

  // How quickly we do framebuffer updates (in ms).  Can't be faster than 16ms which is the graphics task loop time.
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
void GraphicsAnimations::flower(FrameBuffer bbuf) {

  // How quickly we do framebuffer updates (in ms).  Can't be faster than 16ms which is the graphics task loop time.
  uint16_t animatePeriod = 25;
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

//#############################
//  Animated blinking eyeballs
//#############################
void GraphicsAnimations::eyeball(FrameBuffer bbuf) {

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

//########## rendering for each of the marshmallow phases #############
// Just sit there on the stick
void GraphicsAnimations::renderRaw(MarshmallowState& mm, uint32_t t, palette& colors) {
    mm.rotation = 0;
    mm.cy       = 26;
    mm.halfSize = 3;
    mm.color    = colors.white;
}

// Strart toasting - turn brown
void GraphicsAnimations::renderToasting(MarshmallowState& mm, uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.rotation = lerp(0,10,u);
    mm.cy       = 26;
    mm.halfSize = 3;
    lerpColor(mm.color,colors.white, colors.lightbrown, u);
}

// Strart melting/dropping
void GraphicsAnimations::renderMelting(MarshmallowState& mm, uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.rotation = lerp(10,20,u);
    //mm.cy = lerp(26,24,u);
    mm.cy       = 26;
    mm.halfSize = 3;
    lerpColor(mm.color, colors.lightbrown, colors.brown, u);
}

// Drop into the file
void GraphicsAnimations::renderDropping(MarshmallowState& mm, uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.cy = lerp(26,18,u);
    mm.rotation = lerp(20,25,u);
    mm.halfSize = 3;
    lerpColor(mm.color, colors.brown, colors.darkgray, u);
}
// Burned sitting in the fire
void GraphicsAnimations::renderBurnt(MarshmallowState& mm, uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.cy = lerp(18,10,u);
    mm.rotation = lerp(25,29,u);
    mm.halfSize = 3;
    lerpColor(mm.color, colors.darkgray, colors.black, u);
}

// Up in smoke
void GraphicsAnimations::renderSmoke(MarshmallowState& mm, uint32_t t, palette& colors) {
    // check for 0 case so we don't get div by zero error
    float u = (mm.phaseRunTime > 0) ? clamp(t / float(mm.phaseRunTime), 0.0f, 1.0f) : 1.0f;
    mm.cy = lerp(10,44,u);
    //lerpColor(mm.color, colors.white, colors.black, u);
    mm.color = colors.white;
}


// Call the rendering functions based on what state we are in
void GraphicsAnimations::renderMarshmallow(FrameBuffer bbuf, MarshmallowState& mm, uint32_t now, palette& colors) {
  uint32_t phaseElapsed = now - mm.phaseStartTime;

    switch (mm.phase) {
        case MM_RAW:
            renderRaw(mm,phaseElapsed, colors);
            break;

        case MM_TOASTING:
            renderToasting(mm,phaseElapsed,colors);
            break;

        case MM_MELTING:
            renderMelting(mm,phaseElapsed,colors);
            break;

        case MM_DROPPING:
            renderDropping(mm,phaseElapsed,colors);
            break;

        case MM_BURNT:
            renderBurnt(mm,phaseElapsed,colors);
            break;

        case MM_SMOKE:
            renderSmoke(mm,phaseElapsed,colors);
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
  

//####################################
//  Whole globe sized Pacman chomping
//####################################
void GraphicsAnimations::pacman(FrameBuffer bbuf) {

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
void GraphicsAnimations::pacman1(FrameBuffer bbuf) {

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






//###############################################################################
//  Animated Shooting Star
//  We have an active "star" (head) that traces a path into the framebuffer, then
//  we use a fade function to go erase the trails with a defined time-constant.
//  Renders a a bright star shooting across the sphere with the tail fading off
//  behind it.
//###############################################################################
//     Helper functions 
void GraphicsAnimations::fadeFramebuffer(FrameBuffer bbuf, uint8_t decay) {
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

// Set up parameters for multiple shootingStars
void GraphicsAnimations::initShootingStars() {
  for (int i = 0; i < NUM_STARS; i++) {
    gParticles.getStar(i).x  = random(0, COLUMNS);
    gParticles.getStar(i).y  = random(0, ROWS);
    gParticles.getStar(i).vx = random(-30, 30) / 100.0;   // subtle drift
    gParticles.getStar(i).vy = - (random(80, 140) / 100.0);

    gParticles.getStar(i).r = random(0, 255);
    gParticles.getStar(i).g = random(0, 255);
    gParticles.getStar(i).b = random(0, 255);
  }
}

// Main rendering function
void GraphicsAnimations::shootingStar(FrameBuffer bbuf) {

  // Iterate over the entire framebuffer subtracting brightness from each
  // pixel until we fade to black
  fadeFramebuffer(bbuf, 10);   // larger number fades quicker

  for (int i = 0; i < NUM_STARS; i++) {

    Star *s = &gParticles.getStar(i);

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

//##########################################
// Helper functions for the sparkShower
// An animation the floods "sparks" out the
// top of the sphere to cascade down until 
// fading out
//##########################################
void GraphicsAnimations::shiftDown(FrameBuffer bbuf) {
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
void GraphicsAnimations::sparkShower(FrameBuffer bbuf) {

  fadeFramebuffer(bbuf, 7);   // short persistence
  shiftDown(bbuf);          // actual downward motion

  int topRow = ROWS - 1;

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