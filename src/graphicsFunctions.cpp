
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
  
  static int bgColorIndex = 0;
  static int fgColorIndex = 1;
  int colorArrLen = sizeof(colors) / sizeof(colors[0]);

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
  int head = (elapsed * 48) / 3000;      // 0–47 over 3 seconds
  head %= 48;  // Wrap back the start once head reaches the bottom of the Sphere

  // load with R, G, B for index 0,1,2
  const uint8_t r[3] = {255,0,0};
  const uint8_t g[3] = {0,255,0};
  const uint8_t b[3] = {0,0,255};

  // for each row, figure out which color band it lays in
  for (int row = 0; row < 48; row++) {
    int d = (head - row + 48) % 48;   // distance behind the head this current row is
    int colorIndex;
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
  int colorArrLen = sizeof(colors) / sizeof(colors[0]);

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
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
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
    // open mouth is an ellipse.  radiusX is the width, radiusY is the height
    drawEllipse(40, 28, 25, 8, 0, c.black);
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
  drawPacman(41,23,c.yellow,c.black,mouthOpen);

  // Draw the ghosts
  drawGhost(65, 22, c.red, c.black, c.white,jumpUp);
  drawGhost(85, 22, c.blue, c.black, c.white,!jumpUp);
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
    if(animateCount >= 20) {
      animateIncrement = -1;
    }
    if(animateCount <= borderWidth) {
      animateIncrement = 1;
    }
    animateCount += animateIncrement;
  }

  drawDiamond(20, 22, animateCount, animateCount, c.yellow);
  drawDiamond(20, 22, animateCount-borderWidth, animateCount-borderWidth, c.purple);

  drawDiamond(60, 22, animateCount, animateCount, c.blue);
  drawDiamond(60, 22, animateCount-borderWidth, animateCount-borderWidth, c.yellow);

  drawDiamond(100, 22, animateCount, animateCount, c.purple);
  drawDiamond(100, 22, animateCount-borderWidth, animateCount-borderWidth, c.yellow);
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
    if(animateCount >= 47) {
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
//  Right twist Spiral around the Sphere
//#######################################
void fillBB_spiralR(){

  constexpr uint32_t spiralRevPeriod = 3000; //in mS 
  constexpr int K = 1;  // Spiral twist factor
  constexpr int numSpirals = 6;
  constexpr int thickness = 8;  // How many pixels wide are the spirals
  static int fg0ColorIndex = 0;
  static int fg1ColorIndex = 1;
  RGB bgColor = {0,0,0};
  int colorArrLen = sizeof(spiralColors) / sizeof(spiralColors[0]);

  static uint32_t phase = 0;
  static uint32_t lastColorSwitchTime = 0;
  uint32_t now = millis();

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
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      writePixel(col, row, bgColor);
    }
  }
  drawSpiral(phase, K, numSpirals, thickness, spiralColors[fg0ColorIndex]);

}

//#######################################
//  Left twist Spiral around the Sphere
//#######################################
void fillBB_spiralL(){

  constexpr uint32_t spiralRevPeriod = 3000; //in mS 
  constexpr int K = 1;  // Spiral twist factor
  constexpr int numSpirals = 8;
  constexpr int thickness = 8;  // How many pixels wide are the spirals
  static int fg0ColorIndex = 0;
  static int fg1ColorIndex = 1;
  RGB bgColor = {0,0,0};
  int colorArrLen = sizeof(spiralColors) / sizeof(spiralColors[0]);

  static uint32_t phase = 0;
  static uint32_t lastColorSwitchTime = 0;
  uint32_t now = millis();

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
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      writePixel(col, row, bgColor);
    }
  }
  drawSpiral(phase, -K, numSpirals, thickness, spiralColors[fg0ColorIndex]);

}

//##################################
//  Double Spiral around the Sphere
//##################################
void fillBB_spiralD(){

  constexpr uint32_t spiralRevPeriod = 3000; //in mS 
  constexpr int K = 1;  // Spiral twist factor
  constexpr int numSpirals = 6;
  constexpr int thickness = 3;  // How many pixels wide are the spirals
  static int fg0ColorIndex = 0;
  static int fg1ColorIndex = 1;
  RGB bgColor = {0,0,0};
  int colorArrLen = sizeof(spiralColors) / sizeof(spiralColors[0]);

  static uint32_t phase = 0;
  static uint32_t lastColorSwitchTime = 0;
  uint32_t now = millis();

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
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      writePixel(col, row, bgColor);
    }
  }
  drawSpiral(phase, K, numSpirals, thickness, spiralColors[fg0ColorIndex]);
  drawSpiral(phase, -K, numSpirals, thickness, spiralColors[fg1ColorIndex]);

}

//#############################
//  Animated blinking eyeball
//#############################
void fillBB_eyeball() {

  // How quickly we do framebuffer updates (in ms)
  constexpr uint16_t animatePeriod = 1000;
  constexpr uint16_t blinkPeriod = 5;  // how fast to blink
  constexpr int ANIMATE_CYCLES_UNTIL_TRIGTGER = 1;  // how often to blink
  static int blinkTrigger = ANIMATE_CYCLES_UNTIL_TRIGTGER;  
  static int blinkToRow = 0;  // How far down the blink is
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
      blinkToRow+=4;
      if(blinkToRow >= 40) {
        whichEyeToBlink = int(random(3));
        blinkTrigger = ANIMATE_CYCLES_UNTIL_TRIGTGER; 
        blinkToRow = 0;
      }
    }
  }
  // Draw Eyeball
  drawEyeball(20, 22, 14, c.mediumblue, c.black, c.mediumgray, position[posIndex] );
  drawEyeball(60, 22, 14, c.olivegreen, c.black, c.mediumgray, position[posIndex] );
  drawEyeball(100, 22, 14, c.brown, c.black, c.mediumgray, position[posIndex] );

  if(blinkTrigger < 0) {
    // Mask the eyeball from the top down in flesh tone (like an eyelid)
    if(whichEyeToBlink == 0) {
      drawRect(6,blinkToRow,34,0,c.black);
    } else if(whichEyeToBlink == 1) {
      drawRect(46,blinkToRow,74,0,c.black);
    } else {
      drawRect(86,blinkToRow,114,0,c.black);
    }
  }
}

//#############################
//  Animated Marshmallow Roast
//#############################
void fillBB_pinecrest() {

  // Get the colors
  static palette c;
  RGB flames[] = {c.red,c.yellow};

  // Use time to control the animation
  uint32_t now = millis();
  static uint32_t lastTick = 0;
  static uint32_t animationCount = 0;

  // Base animation clock (fastest tick)
  if (now - lastTick >= 50) {
      uint32_t ticks = (now - lastTick) / 50;
      animationCount += ticks;
      lastTick += ticks * 50;
  }

  // Fill the background
  for (int col = 0; col < COLUMNS; col++) {
    for (int row = 0; row < ROWS; row++) {
      writePixel(col, row, c.black);
    }
  }


  // roasting stick
  drawLine(63,22,90,29,1,c.darkgray);  

  // Marshmallow starts out square, then melts and drops into the fire
  static int mmCx;
  static int mmCy;
  static int rotate;
  static int halfsize;

  if(animationCount < 100) {
    mmCx = 61; mmCy = 20; rotate= 0; halfsize= 3;
  } else if(animationCount >= 100 && animationCount < 110) {
    mmCx = 61; mmCy = 21; rotate= -5; halfsize= 3;
  } else if(animationCount >= 110 && animationCount < 120) {
    mmCx = 61; mmCy = 21; rotate= -10; halfsize= 3;
  } else if(animationCount >= 120 && animationCount < 130) {
    mmCx = 61; mmCy = 21; rotate= -14; halfsize= 3;
  } else if(animationCount >= 130 && animationCount < 140) {
    mmCx = 61; mmCy = 22; rotate= -18; halfsize= 3;
  } else if(animationCount >= 140 && animationCount < 150) {
    mmCx = 61; mmCy = 22; rotate= -25; halfsize= 3;
  } else if(animationCount >= 150 && animationCount < 160) {
    mmCx = 61; mmCy = 23; rotate= -25; halfsize= 3;
  } else if(animationCount >= 160 && animationCount < 170) {
    mmCx = 61; mmCy = 23; rotate= -25; halfsize= 3;
  } else if(animationCount >= 170 && animationCount < 180) {
    mmCx = 61; mmCy = 23; rotate= -25; halfsize= 3;
  } else if(animationCount >= 180 && animationCount < 190) {
    mmCx = 61; mmCy = 24; rotate= -47; halfsize= 2;
  } else if(animationCount >= 190 && animationCount < 201) {
    mmCx = 61; mmCy = 24; rotate= -47; halfsize= 2;
  } else if(animationCount >= 201 && animationCount < 206) {
    mmCx = 61; mmCy = 24; rotate= -47; halfsize= 2;
  } else if(animationCount >= 206 && animationCount < 211) {
    mmCx = 61; mmCy = 25; rotate= -47; halfsize= 2;
  } else if(animationCount >= 211 && animationCount < 216) {
    mmCx = 61; mmCy = 27; rotate= -47; halfsize= 2;
  } else if(animationCount >= 216 && animationCount < 221) {
    mmCx = 61; mmCy = 29; rotate= -47; halfsize= 2;
  } else if(animationCount >= 221 && animationCount < 224) {
    mmCx = 61; mmCy = 31; rotate= -47; halfsize= 2;
  } else if(animationCount >= 224 && animationCount < 227) {
    mmCx = 61; mmCy = 32; rotate= -47; halfsize= 2;
  } else if(animationCount >= 227 && animationCount < 230) {
    mmCx = 61; mmCy = 33; rotate= -47; halfsize= 2;
  } else if(animationCount >= 227 && animationCount < 229) {
    mmCx = 61; mmCy = 34; rotate= -47; halfsize= 2;
  } else if(animationCount >= 229 && animationCount < 231) {
    mmCx = 61; mmCy = 35; rotate= -47; halfsize= 2;
  } else if(animationCount >= 231) {
    mmCx = 61; mmCy = 36; rotate= -47; halfsize= 2;
  }
  drawQuad(mmCx-halfsize,mmCy+halfsize,mmCx-halfsize,mmCy-halfsize,mmCx+halfsize,mmCy-halfsize,mmCx+halfsize,mmCy+halfsize,rotate,c.white);

  // campfire
  drawTriangle({45,40},{50,random(26,31)},{55,40},flames[random(2)]);
  drawTriangle({55,40},{60,random(26,31)},{65,40},flames[random(2)]);
  drawTriangle({65,40},{70,random(26,31)},{75,40},flames[random(2)]);
  drawTriangle({50,40},{55,random(26,31)},{60,40},flames[random(2)]);  
  drawTriangle({60,40},{65,random(26,31)},{70,40},flames[random(2)]);

  // the owl
  //drawOwl(90,25,0);
   drawArc({90,30},{85,25},{90,20},1,c.green);
   drawArc({100,20},{105,25},{100,30},2,c.yellow);
   drawArc({90,20},{100,10},{110,20},3,c.red);
   drawArc({110,30},{100,40},{90,30},4,c.blue);

  //Serial.printf("x: %d  y: %d\n",x,y);
  //Serial.printf("cx: %.1f  cy: %.1f  r: %.1f,  theta1: %.1f  theta2: %.1f  theta3: %.1f\n",cx,cy,r,theta1,theta2,theta3);

  // PINECREST
  font_7x5 f;
  drawLetter(f.P,0 ,21, c.black, c.green);
  drawLetter(f.I,5 ,21, c.black, c.green);
  drawLetter(f.N,10 ,21, c.black, c.green);
  drawLetter(f.E,15 ,21, c.black, c.green);
  drawLetter(f.C,20 ,21, c.black, c.green);
  drawLetter(f.R,25 ,21, c.black, c.green);
  drawLetter(f.E,30 ,21, c.black, c.green);
  drawLetter(f.S,35 ,21, c.black, c.green);
  drawLetter(f.T,40 ,21, c.black, c.green);

  drawLetter(f.N2,15 ,29, c.black, c.green);
  drawLetter(f.N0,20 ,29, c.black, c.green);
  drawLetter(f.N2,25 ,29, c.black, c.green);
  drawLetter(f.N6,30 ,29, c.black, c.green);


  // Reset the scene
  if(animationCount > 250) {
    animationCount = 0;
    mmCx = 61;
    mmCy = 20;
    halfsize = 3;
    rotate=0;
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
void drawSpiral(uint32_t phase, int K, int numSpirals, int thickness, const struct RGB& color) {
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
void drawPacman(int centerX, int centerY, const struct RGB& bodyColor, const struct RGB& bgColor, bool mouthOpen) {

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
//  Draw Owl
//######################
void drawOwl(int centerX, int centerY,  bool blink) {
  int dY;
  palette c;  // Get color palette

  if(blink) {
    //close eyes
  } else {
    // open eyes
  }
  //head
  drawEllipse(centerX, centerY+10, 10, 7, 0, c.brown);

  // body
  drawEllipse(centerX-10, centerY+5, 5, 15, -15, c.brown);
  drawEllipse(centerX-15, centerY+5, 5, 15, -15, c.brown);
  drawEllipse(centerX, centerY+5, 5, 15, -15, c.brown);
  drawEllipse(centerX+5, centerY+5, 5, 15, -15, c.brown);
  drawEllipse(centerX+10, centerY+5, 5, 15, -15, c.brown);
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
  drawRect(centerX-5, centerY+6+dY, centerX+5, centerY+dY, bodyColor); //body
  drawCircle(centerX-2, centerY+dY, 1, eyeColor);  //left eye
  drawCircle(centerX+2, centerY+dY, 1, eyeColor);  //right eye
  drawTriangle({centerX-2,centerY+5+dY},{centerX-3,centerY+6+dY},{centerX-1,centerY+6+dY},bgColor); //right bottom cutout
  drawTriangle({centerX+2,centerY+5+dY},{centerX+3,centerY+6+dY},{centerX+1,centerY+6+dY},bgColor); //left bottom cutout
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
  if(move == UP) {dX=0; dY = -5;}
  if(move == DOWN) {dX=0; dY = 5;}

  drawCircle(centerX,centerY, radius, fgColor);  // White of the eye
  drawCircle(centerX+dX, centerY+dY, radius/2, eyeColor);  //iris
  drawCircle(centerX+dX, centerY+dY, radius/4, bgColor);   //pupil
}

//####################################################
//  Draw Letters.  Read fonts from graphicsFunctions.h
//  Letters are 7x5 caps.
//####################################################
void drawLetter(uint8_t (*letter)[5], int llX ,int llY, const struct RGB& bgColor, const struct RGB& fgColor){
  for (int col = 0; col < 5; col++) {
    for(int row = 0; row < 7; row++) {
      if(letter[row][col] == 1) {
        writePixel(col+llX, llY-6 + row, fgColor);
      } else {
        writePixel(col+llX, llY-6 + row, bgColor);
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

//##########################################################
//  Draw filled rect with specified  ll/ur points and color
//##########################################################
void drawRect(int llX, int llY, int urX, int urY, const struct RGB& color) {
  for (int col = llX; col <= urX; col++) {
    for(int row = urY; row <= llY; row++) {
      writePixel(col, row, color);
    }
  }
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

