
#include <graphicsFunctions.h>

// Each function can be selected by the user from the rotary-encoder/OLED interface
//
// The functions take the absolute time (millis()) and compute animation cycles, phase, 
// animation speed, etc. to compute pixel RGB values anywhere on the sphere.  Then we just
// iterate across the available rows/columns and compute those pixels (basically a coarse
// mapping of the sphere data to the sparse LED matrix overlayed onto it).

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

  // Pattern of how we want the boundary edge to look (just "paint" one half and we will tile it to both sides of the Sphere
  // the numbers are the numbe of rows the column will modify its head-pointer by.  minus means up (the sphere rows start a 0 on top).  Pos means down.
  static const int8_t waveLUT[COLUMNS/2] = {
    -1,-1,0,0,-1,0,5,6,5,-3,-2,0,0,-1,-1,0,1,0,5,6,5,-3,-4,-3,0,0,-3,-4,-3,0,  1,0,0,0,1,10,11,10,0,-1,-2,-3,-2,-1,0,0,-1,0,5,8,5,1,0,-3,-3,0,-2,-1,-1,0,
  };
  
  static uint8_t bgColorIndex = 0;
  static uint8_t fgColorIndex = 1;
  uint8_t colorArrLen = sizeof(colors) / sizeof(colors[0]);

  // Create instances for the foreground and background color
  static RGB fgColor, bgColor;
  static uint32_t lastCycle = 0;

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
  head %= 48;  // Wrap back the start once head reaches the bottom of the Sphere

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
  uint8_t colorBright = (1 - cos(phase)) * 0.5 * 255;

  // Now map the brightness based on the position
  for (int col = 0; col < COLUMNS; col++) {
    float theta = (col / float(COLUMNS)) * 2*PI;
    for (int row = 0; row < ROWS; row++) {
      float phi = (row / float(ROWS)) * PI - PI/2;

      uint8_t bright = uint8_t((sin(theta * 6 + animatePhase) * 0.5f + 0.5f) * colorBright);

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
  static uint8_t bg0ColorIndex = 0;
  static uint8_t fg0ColorIndex = 1;
  static uint8_t bg1ColorIndex = 2;
  static uint8_t fg1ColorIndex = 3;

  // colors defined in graphicsFunctions.h
  uint8_t colorArrLen = sizeof(colors) / sizeof(colors[0]);

  // Use time to control the frequency of color changes
  uint16_t colorPeriod = 2000;
  uint32_t elapsed = millis();

  // What cycle are we in?
  uint32_t cycle = elapsed / colorPeriod; 
  static uint32_t lastCycle = 0;

  // The advancing line where the color1 pair replaces color0 pair.  Wrap back to 0 after advancing past COLUMNS
  uint8_t head = (elapsed * COLUMNS) / colorPeriod % COLUMNS;  

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
  for (uint8_t col = 0; col < COLUMNS; col++) {
    uint8_t d = (head - col + COLUMNS) % COLUMNS;   // distance behind the head this current col is
    uint8_t fgIndex, bgIndex;

    // Pick the color pair based on where we are relative to the moving head dividing line
    if (d > 0 && d <= head) {
      fgIndex = fg1ColorIndex;
      bgIndex = bg1ColorIndex;
    } else {  
      fgIndex = fg0ColorIndex;
      bgIndex = bg0ColorIndex;
    }
    for (uint8_t row = 0; row < ROWS; row++) {

      // shift by 3 so we "coarseify" the xor to happen across eight row/col bands
      bool fg = ((row >> 3) ^ (col >> 3)) & 1;
      RGB color;
      if(fg) {
        color = colors[fgIndex];
      } else { 
        color = colors[bgIndex];
      }
      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = color.r;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = color.g;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = color.b;
    }
  }
}

//##################
//  Pacman chomping
//##################
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
  for (uint8_t col = 0; col < COLUMNS; col++) {
    for (uint8_t row = 0; row < ROWS; row++) {
      writePixel(col, row, c.yellow);
    }
  }

  // Draw the left eye (a circle with a notch)
  drawCircle(30,15,4,c.black);
  drawTriangle({28,15}, {20,5}, {20,25}, c.yellow);

  // Draw the right eye
  drawCircle(50,15,4,c.black);
  drawTriangle({52,15}, {60,5}, {60,25}, c.yellow);

  // Animate the mouth by drawing open/close based on the cycle we are in
  
  static bool open = false;
  if(cycle != lastCycle) {
    open = !open;
    lastCycle = cycle;
  }

  if(open) {
    // open mouth is an oval.  radiusA is the width, radiusB is the height
    drawOval(40, 28, 25, 8, c.black);
  } else {

    // Draw the closed mouth (simple horizontal line)
    drawRect(15, 28, 65, 28, c.black);
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
  for (uint8_t col = 0; col < COLUMNS; col++) {
    for (uint8_t row = 0; row < ROWS; row++) {
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
  drawPacman(41,23,c.yellow,c.black,mouthOpen);

  // Draw the ghosts
  drawGhost(65, 22, c.red, c.black, c.white,jumpUp);
  drawGhost(85, 22, c.lightBlue, c.black, c.white,!jumpUp);
  drawGhost(105, 22, c.orange, c.black, c.white,jumpUp);

  // Draw the Pacman food dots
  drawCircle(5, 24, 2, c.yellow);
  drawCircle(15, 24, 2, c.yellow);
  drawCircle(25, 24, 2, c.yellow);

}


//#########################
//  Expanding Diamonds
//#########################
void fillBB_diamond() {

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
  for (uint8_t col = 0; col < COLUMNS; col++) {
    for (uint8_t row = 0; row < ROWS; row++) {
      writePixel(col, row, c.black);
    }
  }
}


//#############################
//  Spiral around the Sphere
//#############################
constexpr RGB spiralColors[] = {
                {255,0,0},
                {0,255,0},
                {0,0,255},
              };

void fillBB_spiral(){

  constexpr uint32_t spiralRevPeriod = 5000; //in mS 
  constexpr uint8_t K = 1;  // Spiral twist factor
  constexpr uint8_t numSpirals = 6;
  constexpr uint8_t thickness = 3;  // How many pixels wide are the spirals
  static uint8_t fg0ColorIndex = 0;
  static uint8_t fg1ColorIndex = 1;
  RGB bgColor = {0,0,0};
  uint8_t colorArrLen = sizeof(spiralColors) / sizeof(spiralColors[0]);

  // phase tells us where in a revolution we currently are.  Use a fixed-point accumulator.  
  // It always increments by one when we pass a period/COLUMNS point
  // Using an accumulator fixes the integer quantization + wrap issues that my original 
  // uint32_t phase = uint32_t(t * COLUMNS/spiralRevPeriod) % COLUMNS; method had.
  // We need to accumulate all the fractional phase until we get to a whole column jump.
  static uint32_t phase = 0;
  static uint32_t remainder = 0;      // remainder in "ms·columns"
  static uint32_t lastT = 0;
  
  static uint32_t lastColorSwitchTime = 0;
  uint32_t now = millis();
  uint32_t dt = now - lastT;
  lastT = now;
  
  // accumulate fractional progress
  remainder += dt * COLUMNS;
  
  // extract whole columns
  uint32_t step = remainder / spiralRevPeriod;
  remainder %= spiralRevPeriod;
  
  // advance phase
  //dlf Temporarily stop phase shifting (just use scroll mode) until I figure out phase bug.
  //phase = (phase + step) % COLUMNS;

  // change color every rev cycle
  if (now - lastColorSwitchTime >= spiralRevPeriod) {    
    lastColorSwitchTime = now;
    fg0ColorIndex = fg1ColorIndex;
    fg1ColorIndex += 1;

    if(fg1ColorIndex == colorArrLen) {
       fg1ColorIndex = 0;
    }
  }

  // Fill the background
  for (uint8_t col = 0; col < COLUMNS; col++) {
    for (uint8_t row = 0; row < ROWS; row++) {
      writePixel(col, row, bgColor);
    }
  }
  drawSpiral(phase, K, numSpirals, thickness, spiralColors[fg0ColorIndex]);
  drawSpiral(phase, -K, numSpirals, thickness, spiralColors[fg1ColorIndex]);

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
void drawSpiral(uint32_t phase, int K, uint8_t numSpirals, uint8_t thickness, const struct RGB& color) {
  static int pcount = 0;
  int wD,wDF;
  for (int phi = 0; phi < ROWS; ++phi) { 
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
void drawPacman(uint8_t centerX, uint8_t centerY, const struct RGB& bodyColor, const struct RGB& bgColor, bool mouthOpen) {

  // head
  drawCircle(centerX,centerY,11,bodyColor);

  // Draw Pacman's eye
  drawCircle(centerX-2,centerY-6,2,bgColor);

  // animate the mouth
  if(mouthOpen) {
    // open mouth is a wedge since we are looking at a side view
    //drawTriangle({float(centerX-12),float(centerY-7)}, {float(centerX-12),float(centerY+7)}, {float(centerX),float(centerY)}, bgColor);
    drawTriangle({centerX-12,centerY-7}, {centerX-12,centerY+7}, {centerX,centerY}, bgColor);
  } else {
    // Draw the closed mouth (simple horizontal line)
    drawRect(centerX-11, centerY, centerX-2, centerY, bgColor);
  }
}

//######################
//  Draw Pacman Ghost
//######################
void drawGhost(uint8_t centerX, uint8_t centerY, const struct RGB& bodyColor, const struct RGB& bgColor, const struct RGB& eyeColor, bool jumpUp) {
  uint8_t dY;
  if(jumpUp) {
    dY = 3;
  } else {
    dY = 0;
  }
  drawCircle(centerX,centerY+dY, 5, bodyColor);  // Head
  drawRect(centerX-5, centerY+6+dY, centerX+5, centerY+dY, bodyColor); //body
  drawCircle(centerX-2, centerY+dY, 1, eyeColor);  //left eye
  drawCircle(centerX+2, centerY+dY, 1, eyeColor);  //right eye
  drawTriangle({centerX-2,centerY+5+dY},{centerX-3,centerY+6+dY},{centerX-1,centerY+6+dY},bgColor); //right bottom cutout
  drawTriangle({centerX+2,centerY+5+dY},{centerX+3,centerY+6+dY},{centerX+1,centerY+6+dY},bgColor); //left bottom cutout
  writePixel(centerX-2, centerY+dY, bgColor);  //left iris
  writePixel(centerX+2, centerY+dY, bgColor);  //right iris
}

//####################################################
//  Draw filled circle into the backbuffer at a given 
// center, with a given radius, and given color
//####################################################
void drawCircle(uint8_t centerX, uint8_t centerY, uint8_t radius, const struct RGB& color) {
  uint8_t plusMinus;
  for (uint8_t col = centerX - radius; col <= centerX + radius; col++) {
    plusMinus = sqrt(pow(radius,2) - pow((col - centerX),2));
    for(uint8_t row = centerY - plusMinus; row <= centerY + plusMinus; row++) {
      writePixel(col, row, color);
    }
  }
}

//###########################################################
//  Draw filled oval into the backbuffer at a given 
// center, with a given minior/major radius, and given color
//###########################################################
void drawOval(uint8_t centerX, uint8_t centerY, uint8_t radiusA, uint8_t radiusB, const struct RGB& color) {
  uint8_t plusMinus;
  for (uint8_t col = centerX - radiusA; col <= centerX + radiusA; col++) {
    plusMinus = (float(radiusB)/float(radiusA)) * sqrt(pow(float(radiusA),2.0) - pow((col - centerX),2));
    for(uint8_t row = centerY - plusMinus; row <= centerY + plusMinus; row++) {
      writePixel(col, row, color);
    }
  }
}

//###########################################################
//  Draw filled diamond to the backbuffer at a given 
// center, with a given minior/major extents and given color
//###########################################################
void drawDiamond(uint8_t centerX, uint8_t centerY, uint8_t extentX, uint8_t extentY, const struct RGB& color) {
  uint8_t plusMinus;
  for (uint8_t col = centerX - extentX; col <= centerX + extentX; col++) {
    //plusMinus =  y=mx+b; 
    for(uint8_t row = centerY - plusMinus; row <= centerY + plusMinus; row++) {
      writePixel(col, row, color);
    }
  }
}

//##########################################################
//  Draw filled rect with specified  ll/ur points and color
//##########################################################
void drawRect(uint8_t llX, uint8_t llY, uint8_t urX, uint8_t urY, const struct RGB& color) {
  for (uint8_t col = llX; col <= urX; col++) {
    for(uint8_t row = urY; row <= llY; row++) {
      writePixel(col, row, color);
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
  uint8_t minX, maxX, minY, maxY;
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
  for (uint8_t col = minX; col <= maxX; col++) {
    for(uint8_t row = minY; row <= maxY; row++) {
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
void writePixel(uint8_t col, uint8_t row, const struct RGB& color) {
      backBuffer[(row * COLUMNS * 3) + (col * 3)]     = color.r;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 1)] = color.g;
      backBuffer[(row * COLUMNS * 3) + (col * 3 + 2)] = color.b;
}
