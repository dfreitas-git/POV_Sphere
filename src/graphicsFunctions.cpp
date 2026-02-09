
#include <graphicsFunctions.h>


// Each function can be selected by the user from the rotary-encoder/OLED interface
//
// The functions take the absolute time (millis()) and compute animation cycles, phase, 
// animation speed, etc. to compute pixel RGB values anywhere on the sphere.  Then we just
// iterate across the available rows/columns and compute those pixels (basically a coarse
// mapping of the sphere data to the sparse LED matrix overlayed onto it).


// Global definitions
Star stars[NUM_STARS];

// generally helpful utility functions

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

// clear the specified framebuffer
void clearFrameBuffer(uint8_t* frameBuffer) {
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      int idx = (row * COLUMNS + col) * 3;
      frameBuffer[idx]     = 0;
      frameBuffer[idx + 1] = 0;
      frameBuffer[idx + 2] = 0;
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
      // Reverse the rows since Gimp stores row 0 at the top and our framebuffer starts with 0 at the bottom.
      backBuffer[((ROWS-1-row) * COLUMNS * 3) + (col * 3)] = r8;
      backBuffer[((ROWS-1-row) * COLUMNS * 3) + (col * 3 + 1)] = g8;
      backBuffer[((ROWS-1-row) * COLUMNS * 3) + (col * 3 + 2)] = b8;

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
  int colorBright = (1 - cos(phase)) * 0.5 * 255;

  // Precompute sin of phi to keep it out of the inner loop
  float rowSin[ROWS];
  for (int row = 0; row < ROWS; row++) {
    float phi = ((float(row) / float(ROWS)) * PI) - PI/2.0;
    rowSin[row] = sin(phi * 8 + animatePhase);
  }
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      int bright = uint8_t((rowSin[row] * 0.5f + 0.5f) * colorBright);
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

      int bright = uint8_t((sin(theta * 6 + animatePhase) * 0.5f + 0.5f) * colorBright);

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
      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = color.r;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = color.g;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = color.b;
    }
  }
}

//####################################
//  Whole globe sized Pacman chomping
//####################################
void fillBB_pacman() {

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
      writePixel(col, row, c.yellow);
    }
  }

  // Draw the left eye (a circle with a notch)
  drawCircle(30,32,4,c.black);
  drawTriangle({28,32}, {20,42}, {20,22}, c.yellow);

  // Draw the right eye
  drawCircle(50,32,4,c.black);
  drawTriangle({52,32}, {60,42}, {60,22}, c.yellow);

  // Animate the mouth by drawing open/close based on the cycle we are in
  
  static bool open = false;
  if(cycle != lastCycle) {
    open = !open;
    lastCycle = cycle;
  }

  if(open) {
    // open mouth is an ellipse.  radiusX is the width, radiusY is the height
    drawEllipse(40, 19, 25, 8, 0, c.black);
  } else {

    // Draw the closed mouth (simple horizontal line)
    drawRect(15, 19, 65, 19, 0, c.black);
  }
}

//#########################
//  Pacman chasing ghosts
//#########################
void fillBB_pacman1() {

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
      writePixel(col, row, c.black);
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
  drawPacman(41,24,c.yellow,c.black,mouthOpen);

  // Draw the ghosts
  drawGhost(65, 25, c.red, c.black, c.white,jumpUp);
  drawGhost(85, 25, c.blue, c.black, c.white,!jumpUp);
  drawGhost(105, 25, c.orange, c.black, c.white,jumpUp);

  // Draw the Pacman food dots
  drawCircle(5, 23, 2, c.yellow);
  drawCircle(15, 23, 2, c.yellow);
  drawCircle(25, 23, 2, c.yellow);
}


//#########################
//  Expanding Diamonds
//#########################
void fillBB_diamond() {

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
      writePixel(col, row, c.black);
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

  drawDiamond(20, 24, animateCount, animateCount, c.yellow);
  drawDiamond(20, 24, animateCount-borderWidth, animateCount-borderWidth, c.purple);

  drawDiamond(60, 24, animateCount, animateCount, c.blue);
  drawDiamond(60, 24, animateCount-borderWidth, animateCount-borderWidth, c.yellow);

  drawDiamond(100, 24, animateCount, animateCount, c.purple);
  drawDiamond(100, 24, animateCount-borderWidth, animateCount-borderWidth, c.yellow);
}

//#########################
//  Expanding Flower
//#########################
void fillBB_flower() {

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
      writePixel(col, row, c.black);
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

  // From top to bottome
  drawTriangle({0,0},{20,animateCount},{39,0}, c.red);
  drawTriangle({40,0},{60,animateCount},{79,0}, c.blue);
  drawTriangle({80,0},{100,animateCount},{119,0}, c.green);

  // From bottom to top
  drawTriangle({0,47},{0,47-animateCount},{19,47}, c.blue);
  drawTriangle({20,47},{40,47-animateCount},{59,47}, c.green);
  drawTriangle({60,47},{80,47-animateCount},{99,47}, c.red);
  drawTriangle({100,47},{119,47-animateCount},{119,47}, c.blue);

}

//#######################################
//  Main spiral draw engine.  
//#######################################
void cycle_spiral(int numSpirals, int thickness, int twist){

  // How long between color changes
  constexpr uint32_t spiralRevPeriod = 3000; //in mS 

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

  // Change color every rev cycle
  if (now - lastColorSwitchTime >= spiralRevPeriod) {    
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
      writePixel(col, row, bgColor);
    }
  }
  drawSpiral(phase, -twist, numSpirals, thickness, animateCount, spiralColors[fg0ColorIndex]);
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

  // Change color every rev cycle
  if (now - lastColorSwitchTime >= spiralRevPeriod) {    
    lastColorSwitchTime = now;
    fg0ColorIndex = fg1ColorIndex;
    fg1ColorIndex += 1;

    if(fg1ColorIndex == colorArrLen) {
       fg1ColorIndex = 0;
    }
  }

  // What animate cycle are we in?
  uint32_t cycle = now / animatePeriod; 
  static uint32_t lastCycle = 0;

  // Increment the animate count when we see a cycle transition
  // We increment up and down so the pattern rises/falls
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

  // Fill the background
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      writePixel(col, row, bgColor);
    }
  }
  drawSpiral(phase, K, numSpirals, thickness, animateCount, spiralColors[fg0ColorIndex]);
  drawSpiral(phase, -K, numSpirals, thickness, ROWS-1, spiralColors[fg1ColorIndex]);

}

//#############################
//  Animated blinking eyeball
//#############################
void fillBB_eyeball() {

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
      writePixel(col, row, c.black);
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
  drawEyeball(20, 25, 14, c.mediumblue, c.black, c.mediumgray, position[posIndex] );
  drawEyeball(60, 25, 14, c.olivegreen, c.black, c.mediumgray, position[posIndex] );
  drawEyeball(100, 25, 14, c.brown, c.black, c.mediumgray, position[posIndex] );

  if(blinkTrigger < 0) {
    // Mask the eyeball from the top down over the blinkPeriod in flesh tone (like an eyelid)
    if(whichEyeToBlink == 0) {
      drawRect(6,blinkToRow,34,47, 0,c.black);
    } else if(whichEyeToBlink == 1) {
      drawRect(46,blinkToRow,74,47, 0,c.black);
    } else {
      drawRect(86,blinkToRow,114,47, 0,c.black);
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
      drawArc({mm.cx,mm.cy-3},{mm.cx-arcDepth,mm.cy},{mm.cx,mm.cy+3},1,mm.color);
      arcDepth = arcDepth * -1;  // Alternate the arc back and forth
    } else {
      drawQuad(mm.cx-mm.halfSize, mm.cy+mm.halfSize, mm.cx-mm.halfSize, mm.cy-mm.halfSize, mm.cx+mm.halfSize, mm.cy-mm.halfSize, mm.cx+mm.halfSize, mm.cy+mm.halfSize, mm.rotation, mm.color);
    }
}

  
//#############################
//  Animated Pincrest Scene
//#############################
void fillBB_pinecrest() {

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
      writePixel(col, row, c.black);
    }
  }

  // campfire
  drawTriangle({50,7},{55,random(16,21)},{60,7},flames[random(2)]);
  drawTriangle({60,7},{65,random(16,21)},{70,7},flames[random(2)]);
  drawTriangle({70,7},{75,random(16,21)},{80,7},flames[random(2)]);
  drawTriangle({55,7},{60,random(16,21)},{65,7},flames[random(2)]);  
  drawTriangle({65,7},{70,random(16,21)},{75,7},flames[random(2)]);


  // roasting stick
  drawLine(71,25,90,18,1,c.gray);  

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
  drawOwl(100, 25, blink, squawk);

  // PINECREST letters
  font_7x7 f;
  drawLetter(f.P,0 ,32, c.black, c.green);
  drawLetter(f.I,7 ,32, c.black, c.green);
  drawLetter(f.N,14 ,32, c.black, c.green);
  drawLetter(f.E,21 ,32, c.black, c.green);
  drawLetter(f.C,28 ,32, c.black, c.green);
  drawLetter(f.R,35 ,32, c.black, c.green);
  drawLetter(f.E,42 ,32, c.black, c.green);
  drawLetter(f.S,49 ,32, c.black, c.green);
  drawLetter(f.T,56 ,32, c.black, c.green);

  drawLetter(f.N2,10 ,18, c.black, c.green);
  drawLetter(f.N0,17 ,18, c.black, c.green);
  drawLetter(f.N2,24 ,18, c.black, c.green);
  drawLetter(f.N6,31 ,18, c.black, c.green);

  // Reset the scene
  if(now > sceneStartTime + 11500) {
    timelineInitialized = false;
    owlNextEvent = 0;
    mmNextEvent = 0;
  }
}


//#############################
//  Animated Shooting Star
//#############################
//     Helper functions 
void fadeFramebuffer(uint8_t decay) {
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      for (int c = 0; c < 3; c++) {
        int idx = ((row * COLUMNS + col) * 3) + c;
        uint8_t v = backBuffer[idx];
        backBuffer[idx] = (v > decay) ? (v - decay) : 0;
      }
    }
  }
}

// Write the leading pixel for the shootingStar
void writeHead(int col, int row, RGB color) {
  int idx = (row * COLUMNS + col) * 3;
  backBuffer[idx]     = color.r;
  backBuffer[idx + 1] = color.g;
  backBuffer[idx + 2] = color.b;
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

  // larger number fades quicker
  fadeFramebuffer(10);   

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
    backBuffer[idx]     = s->r;
    backBuffer[idx + 1] = s->g;
    backBuffer[idx + 2] = s->b;
  }
}

// Helper functions for the sparkShower
void shiftDown() {
  for (int row = 0; row < ROWS - 1; row++) {
    for (int col = 0; col < COLUMNS; col++) {
      int dst = (row * COLUMNS + col) * 3;
      int src = ((row + 1) * COLUMNS + col) * 3;
      backBuffer[dst]     = backBuffer[src];
      backBuffer[dst + 1] = backBuffer[src + 1];
      backBuffer[dst + 2] = backBuffer[src + 2];
    }
  }

  // Clear the top row (we'll re-inject)
  int topRow = ROWS - 1;
  for (int col = 0; col < COLUMNS; col++) {
    int idx = (topRow * COLUMNS + col) * 3;
    backBuffer[idx]     = 0;
    backBuffer[idx + 1] = 0;
    backBuffer[idx + 2] = 0;
  }
}

// Main spark rendering function

void fillBB_sparkShower() {

  fadeFramebuffer(7);   // short persistence
  shiftDown();          // actual downward motion

  int topRow = ROWS - 1;

  for (int col = 0; col < COLUMNS; col++) {

    // emission probability per column
    if (random(0, 1000) < 80) {

      int idx = (topRow * COLUMNS + col) * 3;

      uint8_t v = random(150, 255);

      //backBuffer[idx]     = v;
      //backBuffer[idx + 1] = v;
      //backBuffer[idx + 2] = v;
      backBuffer[idx]     = random(255);
      backBuffer[idx + 1] = random(255);
      backBuffer[idx + 2] = random(255);
    }
  }
}


//#############################################################################
// Functions for graphics primatives (circles, rectangles, triangles, etc.)
//#############################################################################

//###############################################################
//  Draw spiral with spiral k-factor (-/+ for left/right twist), 
// width,  number of spirals, color.  phase is the offset from 
// col 0 so we can revolve the spirals.
//###############################################################
// Helper function to return column distance across 0-COLUMNS wrap boundary
static inline int wrapDist(int a, int b) {
    int d = a - b;
    if (d >  COLUMNS / 2) d -= COLUMNS;
    if (d < -(COLUMNS / 2)) d += COLUMNS;
    return d;
}

void drawSpiral(uint32_t phase, int K, int numSpirals, int thickness, int drawToRow, const struct RGB& color) {
  static int pcount = 0;
  static int advanceCount = 0;
  int wD,wDF;
  for (int phi = 0; phi < drawToRow; ++phi) { 
    int base = (K * phi + phase) % COLUMNS; 
    for (int theta = 0; theta < COLUMNS; ++theta) { 
      for (int i = 0; i < numSpirals; ++i) { 

        // Distributes spirals evenly even when COLUMNS/numSpirals isn’t exact.
        int center = (base + (i * COLUMNS) / numSpirals) % COLUMNS;

        // When theta gets close enough to the center of the current row's spiral position, load the color into it
        if (abs(wrapDist(center, theta)) <= thickness) {
          writePixel(theta,phi, color); 
          break; // don’t overdraw 
        } 
      } 
    } 
  }
}

//######################
//  Draw Pacman 
//######################
void drawPacman(int centerX, int centerY, const struct RGB& bodyColor, const struct RGB& bgColor, bool mouthOpen) {

  // head
  drawCircle(centerX,centerY,11,bodyColor);

  // Draw Pacman's eye
  drawCircle(centerX-2,centerY+6,2,bgColor);

  // animate the mouth
  if(mouthOpen) {
    // open mouth is a wedge since we are looking at a side view
    //drawTriangle({float(centerX-12),float(centerY-7)}, {float(centerX-12),float(centerY+7)}, {float(centerX),float(centerY)}, bgColor);
    drawTriangle({centerX-12,centerY+7}, {centerX-12,centerY-7}, {centerX,centerY}, bgColor);
  } else {
    // Draw the closed mouth (simple horizontal line)
    drawRect(centerX-11, centerY, centerX-2, centerY,  0,bgColor);
  }
}

//######################
//  Draw Owl
//######################
void drawOwl(int centerX, int centerY,  bool blink, bool squawk) {
  palette c;  // Get color palette

  // head
  drawEllipse(centerX, centerY+8, 11, 8, 0, c.brown);

  // ears
  drawTriangle({centerX-11,centerY+10},{centerX-15,centerY+16},{centerX-3,centerY+10},c.brown); 
  drawTriangle({centerX+11,centerY+10},{centerX+15,centerY+16},{centerX+3,centerY+10},c.brown); 

  // eyes
  if(blink) {
    //close eyes
    drawEllipse(centerX-5, centerY+8,4,3,0,c.brown);
    drawEllipse(centerX+5, centerY+8,4,3,0,c.brown);
  } else {
    // open eyes
    drawEllipse(centerX-5, centerY+8,4,3,0,c.white);
    drawEllipse(centerX+5, centerY+8,4,3,0,c.white);
    drawCircle(centerX-5,centerY+8, 2, c.black);  // Left iris
    drawCircle(centerX+5,centerY+8, 2, c.black);  // Right iris
  }
  // eyebrows  (raise them when squawking)
  if(squawk) {
    drawArc({centerX-8,centerY+13},{centerX-5,centerY+15},{centerX-2,centerY+13},1,c.yellow);
    drawArc({centerX+9,centerY+13},{centerX+6,centerY+15},{centerX+3,centerY+13},1,c.yellow);
  } else {
    drawArc({centerX-8,centerY+11},{centerX-5,centerY+13},{centerX-2,centerY+11},1,c.yellow);
    drawArc({centerX+9,centerY+11},{centerX+6,centerY+13},{centerX+3,centerY+11},1,c.yellow);
  }

  // beak
  if(squawk) {
    // open beak
    drawCircle(centerX,centerY+4, 2, c.yellow); 
    drawTriangle({centerX-1,centerY+4},{centerX,centerY+3},{centerX+1,centerY+4},c.yellow); 
    drawCircle(centerX,centerY+4, 1, c.black); 
    drawTriangle({centerX-1,centerY+3},{centerX,centerY+2},{centerX+1,centerY+3},c.black); 
  } else {
    // close beak
    drawCircle(centerX,centerY+4, 2, c.yellow); 
    drawTriangle({centerX-1,centerY+4},{centerX,centerY+3},{centerX+1,centerY+4},c.yellow); 
  }

  // Feet
  drawTriangle({centerX-4,centerY-9},{centerX-5,centerY-13},{centerX-1,centerY-9},c.yellow); 
  drawTriangle({centerX+1,centerY-9},{centerX,centerY-13},{centerX+4,centerY-9},c.yellow); 

  // body
  drawEllipse(centerX-5, centerY-5, 3, 6, 15, c.brown);
  drawEllipse(centerX-2,  centerY-5, 3, 6, 15, c.brown);
  drawEllipse(centerX,    centerY-5, 3, 6, 15, c.brown);
  drawEllipse(centerX+2,  centerY-5, 3, 6, 15, c.brown);
  drawEllipse(centerX+5, centerY-5, 3, 6, 15, c.brown);
  drawArc({centerX+6,centerY-10},{centerX+3,centerY-3},{centerX+5,centerY+2},1,c.black);

  // wing
  drawEllipse(centerX-10, centerY-5, 2, 6, -25, c.brown);
}


//######################
//  Draw Pacman Ghost
//######################
void drawGhost(int centerX, int centerY, const struct RGB& bodyColor, const struct RGB& bgColor, const struct RGB& eyeColor, bool jumpUp) {
  int dY;
  if(jumpUp) {
    dY = 3;
  } else {
    dY = 0;
  }
  drawCircle(centerX,centerY+dY, 5, bodyColor);  // Head
  drawRect(centerX-5, centerY-6+dY, centerX+5, centerY+dY,  0,bodyColor); //body
  drawCircle(centerX-2, centerY+dY, 1, eyeColor);  //left eye
  drawCircle(centerX+2, centerY+dY, 1, eyeColor);  //right eye
  drawTriangle({centerX-2,centerY-5+dY},{centerX-3,centerY-6+dY},{centerX-1,centerY-6+dY},bgColor); //right bottom cutout
  drawTriangle({centerX+2,centerY-5+dY},{centerX+3,centerY-6+dY},{centerX+1,centerY-6+dY},bgColor); //left bottom cutout
  writePixel(centerX-2, centerY+dY, bgColor);  //left iris
  writePixel(centerX+2, centerY+dY, bgColor);  //right iris
}

//########################################
//  Draw Eyeball
// "move" uses an enum CENTER/LEFT/RIGHT/UP/DOWN 
//########################################
void drawEyeball(int centerX, int centerY, int radius, const struct RGB& eyeColor, const struct RGB& bgColor, const struct RGB& fgColor, int move ) {

  int dX,dY;
  if(move == CENTER) { dX=0; dY = 0;}
  if(move == LEFT) { dX=-5; dY = 0;}
  if(move == RIGHT) {dX=5; dY = 0;}
  if(move == UP) {dX=0; dY = 5;}
  if(move == DOWN) {dX=0; dY = -5;}

  drawCircle(centerX,centerY, radius, fgColor);  // White of the eye
  drawCircle(centerX+dX, centerY+dY, radius/2, eyeColor);  //iris
  drawCircle(centerX+dX, centerY+dY, radius/4, bgColor);   //pupil
}

//####################################################
//  Draw Letters.  Read fonts from graphicsFunctions.h
//  Letters are 7x5 caps.
//####################################################
void drawLetter(uint8_t (*letter)[7], int llX ,int llY, const struct RGB& bgColor, const struct RGB& fgColor){
  for (int col = 0; col < 7; col++) {
    for(int row = 0; row < 7; row++) {
      if(letter[row][col] == 1) {
        writePixel(col+llX, llY+6 - row, fgColor);
      } else {
        writePixel(col+llX, llY+6 - row, bgColor);
      }
    }
  }
}
//####################################################
//  Draw filled circle into the backbuffer at a given 
// center, with a given radius, and given color
//####################################################
void drawCircle(int centerX, int centerY, int radius, const struct RGB& color) {
  int plusMinus;
  for (int col = centerX - radius; col <= centerX + radius; col++) {
    plusMinus = sqrt(pow(radius,2) - pow((col - centerX),2));
    for(int row = centerY - plusMinus; row <= centerY + plusMinus; row++) {
      writePixel(col, row, color);
    }
  }
}

//###########################################################
//  Draw filled Ellipse into the backbuffer at a given 
// center, with a given minior/major radius, and given color
// drawEllipse completely written by chatGPT.  Handles rotated 
// ellipses (specify in degrees)
//###########################################################

void drawEllipse( int centerX, int centerY, int radiusX, int radiusY, float rotateDeg, const struct RGB& color) {
    // ---- Precompute rotation ----
    float theta = rotateDeg * 3.14159265f / 180.0f;
    float cosT  = cosf(theta);
    float sinT  = sinf(theta);

    float invA2 = 1.0f / (radiusX * radiusX);
    float invB2 = 1.0f / (radiusY * radiusY);

    // ---- World-space AABB of rotated ellipse ----
    // i.e. the maximun extent the ellipse will take in framebuffer (world) space
    // We will use this when scanning X/Y to be sure we cover all the ellipse points
    float ex = sqrtf((radiusX * cosT) * (radiusX * cosT) +
                     (radiusY * sinT) * (radiusY * sinT));
    float ey = sqrtf((radiusX * sinT) * (radiusX * sinT) +
                     (radiusY * cosT) * (radiusY * cosT));

    int minX = (int)floorf(centerX - ex);
    int maxX = (int)ceilf (centerX + ex);
    int minY = (int)floorf(centerY - ey);
    int maxY = (int)ceilf (centerY + ey);

    // ---- Clamp to framebuffer if needed ----
    // minX = max(minX, 0); etc.

    // ---- Scanline fill with span detection ----
    for (int y = minY; y <= maxY; y++) {

        bool inside = false;
        int  spanStartX = 0;

        float dy = (float)y - centerY;

        for (int x = minX; x <= maxX; x++) {

            float dx = (float)x - centerX;

            // Inverse rotate world → ellipse local
            float xr =  dx * cosT + dy * sinT;
            float yr = -dx * sinT + dy * cosT;

            // Ellipse equation in local space
            // if test is < 1 point is inside ellipse, if > 1 it's outside
            bool nowInside =
                (xr * xr) * invA2 +
                (yr * yr) * invB2 <= 1.0f;

            // We detected the boundary of the ellipse, start filling this row from this x-coord
            if (nowInside && !inside) {
                spanStartX = x;
            }

            // Once we find the other edge, we fill the row from the spanStart to the current x
            if (!nowInside && inside) {
                for (int fx = spanStartX; fx < x; fx++) {
                    writePixel(fx, y, color);
                }
            }

            inside = nowInside;
        }

        // Close span at right edge if the second edge was at maxX
        if (inside) {
            for (int fx = spanStartX; fx <= maxX; fx++) {
                writePixel(fx, y, color);
            }
        }
    }
}


//###########################################################
//  Draw filled diamond to the backbuffer at a given 
// center, with a given minior/major extents and given color
//###########################################################
void drawDiamond(int centerX, int centerY, int extentX, int extentY, const struct RGB& color) {
  int plusMinus;
  int leftX = centerX - extentX;
  int leftY = centerY;
  int rightX = centerX + extentX;
  int rightY = centerY;
  int topX = centerX;
  int topY = centerY - extentY;

  // Draw the left half of the diamond with the positive y=mx (remember the "top" is row-0)
  for (int col = leftX; col <= centerX; col++) {
    plusMinus = (float(leftY-topY)/float(topX-leftX)) * (col-leftX);   // Offset X to run from 0 to centerX
    for(int row = centerY - plusMinus; row <= centerY + plusMinus; row++) {
      writePixel(col, row, color);
    }
  }
  // Draw the right half of the diamond with the negative y=mx
  for (int col = topX; col <= rightX; col++) {
    plusMinus = (float(rightY-topY)/float(rightX-topX)) * (rightX-col);   // Offset X to run rightX to 0
    for(int row = centerY - plusMinus; row <= centerY + plusMinus; row++) {
      writePixel(col, row, color);
    }
  }
}

//####################################################################
//  Draw filled rect with specified  ll/ur points, rotation and color
//####################################################################
void drawRect(int llX, int llY, int urX, int urY, int rotate, const struct RGB& color) {
  // In case they mixed up ll/ur
  if(llX > urX) {
    int tmpX = llX;
    llX = urX;
    urX = tmpX;
  }
  if(llY > urY) {
    int tmpY = llY;
    llY = urY;
    urY = tmpY;
  }
  drawQuad(llX, llY, llX, urY, urX, urY, urX, llY, rotate, color);
}

//################################################################
//  Draw filled quadrilateral with specified four points, rotation 
//  angle (in degrees) and color.  Be sure to specify the points 
//  in clockwise order
//################################################################
void drawQuad(int pt0X, int pt0Y, int pt1X, int pt1Y, int pt2X, int pt2Y, int pt3X, int pt3Y, int rotate, const struct RGB& color) {

  // Store in array so we can iterate during transform
  int points[8] = {pt0X, pt0Y, pt1X, pt1Y, pt2X, pt2Y, pt3X, pt3Y};
  int tp[8];

  // get the rotation angle in radians
  float rad = rotate * PI / 180;

  // get the center coords
  float cx = (pt0X + pt1X + pt2X + pt3X) / 4;
  float cy = (pt0Y + pt1Y + pt2Y + pt3Y) / 4;

  // precompute sin/cos to reduce loop time
  float s = sinf(rad);
  float c = cosf(rad);

  //Rotate points and store into translated point array
  for(int i=0;i<8;i+=2) {
    float dx = points[i] - cx;
    float dy = points[i+1] - cy;
    tp[i] = cx + dx * c - dy * s;
    tp[i+1] = cy + dx * s + dy * c;
  }

  // Break the quad into two triangles.  Use the same point rotation (clockwise)
  // Triangle 1: p0, p1, p2
  // Triangle 2: p0, p2, p3
  drawTriangle({tp[0],tp[1]}, {tp[2],tp[3]}, {tp[4],tp[5]}, color);
  drawTriangle({tp[0],tp[1]}, {tp[4],tp[5]}, {tp[6],tp[7]}, color);

}

//##########################################################
//  Draw arc with three points, thickness and color
//##########################################################
// Helper functions
// Test if angle-b is between the endpoints when we travel counter-clockwise
bool angleBetweenCCW(float a, float b, float c)
{
    // true if b is between a and c when sweeping CCW from a to c
    if (a <= c)
        return (b >= a && b <= c);
    else
        return (b >= a || b <= c);
}
// Write the arc's pixels to the framebuffer
void writeArcPixels(int cx, int cy, int r, float theta, int thickness, RGB color) {

  int half = thickness / 2;
  float ca = cosf(theta);
  float sa = sinf(theta);
  // Make thickness centered around the original arc
  for (int t = -half; t <= half; t++) {
    int x = cx + (r + t) * ca;
    int y = cy + (r + t) * sa;
    writePixel(x, y, color);
  }
}

void drawArc(const Vec2& p1, const Vec2& p2, const Vec2& p3, int thickness, const struct RGB& color) {

  // Get the extents of the points to use for scanning later
  int minX, maxX, minY, maxY;
  minX=p1.x;
  if(p2.x < minX) { minX=p2.x; }
  if(p3.x < minX) { minX=p3.x; }
  maxX=p1.x;
  if(p2.x > maxX) { maxX=p2.x; }
  if(p3.x > maxX) { maxX=p3.x; }

  minY=p1.y;
  if(p2.y < minY) { minY=p2.y; }
  if(p3.y < minY) { minY=p3.y; }
  maxY=p1.x;
  if(p2.y > maxY) { maxY=p2.y; }
  if(p3.y > maxY) { maxY=p3.y; }

  // This is just using the perpendicular bisectors of each line segment to find the center
  // of the circle that would pass through the three points.
  int D = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
  float cx = ((pow(p1.x, 2) + pow(p1.y, 2)) * (p2.y - p3.y) + 
              (pow(p2.x, 2) + pow(p2.y, 2)) * (p3.y - p1.y) + 
              (pow(p3.x, 2) + pow(p3.y, 2)) * (p1.y - p2.y)) / D; 
  
  float cy = ((pow(p1.x, 2) + pow(p1.y, 2)) * (p3.x - p2.x) + 
             (pow(p2.x, 2) + pow(p2.y, 2)) * (p1.x - p3.x) + 
             (pow(p3.x, 2) + pow(p3.y, 2)) * (p2.x - p1.x)) / D; 
  
  // radius is just the center to any of the points
  float r = sqrt(pow(cx-p1.x, 2) + pow(cy-p1.y, 2));
  
  // Get the angles from the center to each of the three points
  // Turn them all positive
  float theta1 = atan2f(p1.y - cy, p1.x - cx);
  float theta2 = atan2f(p2.y - cy, p2.x - cx);
  float theta3 = atan2f(p3.y - cy, p3.x - cx);

  // normalize to all positive angles
  if(theta1 < 0) theta1 += 2 * PI;
  if(theta2 < 0) theta2 += 2 * PI;
  if(theta3 < 0) theta3 += 2 * PI;

  // See if we need to traverse clockwise or counter-clockwise to trace the arc through the three points
  bool ccw = angleBetweenCCW(theta1, theta2, theta3);

  //float step = 1.0f / r;   // ~1 pixel per step
  float step = 0.5f / r;   // Had to reduce to 0.5 as 1.0 ended up with some holes in the thick arcs

  // Sweep the arc in polar (r, theta) and convert back to cartesian
  if (ccw) {
    // Take care of wrapping around 0/360 boundary
    if(theta3 < theta1) {
      theta3 += 2*PI;
    }
    for (float a = theta1; a <= theta3; a += step) {
      writeArcPixels(cx, cy, r, a, thickness, color);
    }
  } else {
    // Take care of wrapping around 0/360 boundary
    if(theta1 < theta3) {
      theta1 += 2*PI;
    }
    for (float a = theta1; a >= theta3; a -= step) {
      writeArcPixels(cx, cy, r, a, thickness, color);
    }
  }
}

//##########################################################
//  Draw line with specified two points, thickness and color
//##########################################################
void drawLine(int pt0X, int pt0Y, int pt1X, int pt1Y, int thickness, const struct RGB& color) {
  float m = float(pt1Y-pt0Y)/float(pt1X-pt0X);
  float b = pt0Y - (m * pt0X);

  int minX, minY, maxX, maxY;
  if(pt0X > pt1X) {
    minX=pt1X;
    maxX=pt0X;
  } else {
    minX=pt0X;
    maxX=pt1X;
  }
  if(pt0Y > pt1Y) {
    minY=pt1Y;
    maxY=pt0Y;
  } else {
    minY=pt0Y;
    maxY=pt1Y;
  }

  for (int col = minX; col <= maxX; col++) {
    for(int row = minY; row <= maxY; row++) {
      float y = (m * float(col)) + b;
      if(abs(float(row) - y) <= thickness) {
        writePixel(col, row, color);
      }
    }
  }
}

//##########################################################
//  Draw triangle with three vertices specified and color
//##########################################################
// cross product to generate normal vectors for testing if a point lies within the triangle
float cross(const Vec2& a, const Vec2& b, const Vec2& c) {
  // Cross product of (b - a) x (c - a)
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// test if point lies within the triangle
bool pointInTriangle(const Vec2& p,
                     const Vec2& v1,
                     const Vec2& v2,
                     const Vec2& v3) {
    float d1 = cross(p, v1, v2);
    float d2 = cross(p, v2, v3);
    float d3 = cross(p, v3, v1);

    bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    // Inside or on an edge if all have the same sign (or zero)
    return !(hasNeg && hasPos);
}

void drawTriangle(const Vec2& v1, const Vec2& v2, const Vec2& v3, const struct RGB& color) {

  // Only scan a rectangle as large as the extents of the triangle
  int minX, maxX, minY, maxY;
  minX=v1.x;
  if(v2.x < minX) { minX=v2.x; }
  if(v3.x < minX) { minX=v3.x; }
  maxX=v1.x;
  if(v2.x > maxX) { maxX=v2.x; }
  if(v3.x > maxX) { maxX=v3.x; }

  minY=v1.y;
  if(v2.y < minY) { minY=v2.y; }
  if(v3.y < minY) { minY=v3.y; }
  maxY=v1.x;
  if(v2.y > maxY) { maxY=v2.y; }
  if(v3.y > maxY) { maxY=v3.y; }

  Vec2 pt;
  for (int col = minX; col <= maxX; col++) {
    for(int row = minY; row <= maxY; row++) {
      pt.x = col;
      pt.y = row;
      if(pointInTriangle(pt, v1, v2, v3)) {
        writePixel(col, row, color);
      }
    }
  }
}

//##################################################
//  Write pixel to backBuffer at a given location
//  We use three bytes per pixel for RGB
//##################################################
//void writePixel(int col, int row, const struct RGB& color) {
//  // Be sure they are in-bounds and not negative
//  if(((col >= 0) && (col < COLUMNS)) && ((row >= 0) && (row < ROWS))) {
//      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = color.r;
//      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = color.g;
//      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = color.b;
//  }
//}

void writePixel(int col, int row, const struct RGB& color) {
  // Be sure they are in-bounds and not negative
  if ((unsigned)col < COLUMNS && (unsigned)row < ROWS) {
      int idx = (row * COLUMNS + col) * 3;
      backBuffer[idx]     = color.r;
      backBuffer[idx + 1] = color.g;
      backBuffer[idx + 2] = color.b;
  } else {
      outOfBoundsPixelCount++;
      Serial.printf("writePixel bounds errors: %d\n",outOfBoundsPixelCount);
  }
}

