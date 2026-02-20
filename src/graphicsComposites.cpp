
#include <graphicsPrimitives.h>
#include <graphicsComposites.h>
#include <graphicsGlobals.h>
#include <fonts.h>

//########################################
//  Draw Eyeball
// "move" uses an enum CENTER/LEFT/RIGHT/UP/DOWN 
//########################################
void GraphicsComposites::drawEyeball(FrameBuffer bbuf, int centerX, int centerY, int radius, const struct RGB& eyeColor, const struct RGB& bgColor, const struct RGB& fgColor, int move ) {

  int dX,dY;
  if(move == CENTER) { dX=0; dY = 0;}
  if(move == LEFT) { dX=-5; dY = 0;}
  if(move == RIGHT) {dX=5; dY = 0;}
  if(move == UP) {dX=0; dY = 5;}
  if(move == DOWN) {dX=0; dY = -5;}

  gPrim.drawCircle(bbuf, centerX,centerY, radius, fgColor);  // White of the eye
  gPrim.drawCircle(bbuf, centerX+dX, centerY+dY, radius/2, eyeColor);  //iris
  gPrim.drawCircle(bbuf, centerX+dX, centerY+dY, radius/4, bgColor);   //pupil
}

//######################
//  Draw Pacman Ghost
//######################
void GraphicsComposites::drawGhost(FrameBuffer bbuf, int centerX, int centerY, const struct RGB& bodyColor, const struct RGB& bgColor, const struct RGB& eyeColor, bool jumpUp) {
  int dY;
  if(jumpUp) {
    dY = 3;
  } else {
    dY = 0;
  }
  gPrim.drawCircle(bbuf, centerX,centerY+dY, 5, bodyColor);  // Head
  gPrim.drawRect(bbuf, centerX-5, centerY-6+dY, centerX+5, centerY+dY,  0,bodyColor); //body
  gPrim.drawCircle(bbuf, centerX-2, centerY+dY, 1, eyeColor);  //left eye
  gPrim.drawCircle(bbuf, centerX+2, centerY+dY, 1, eyeColor);  //right eye
  gPrim.drawTriangle(bbuf, {centerX-2,centerY-5+dY},{centerX-3,centerY-6+dY},{centerX-1,centerY-6+dY},bgColor); //right bottom cutout
  gPrim.drawTriangle(bbuf, {centerX+2,centerY-5+dY},{centerX+3,centerY-6+dY},{centerX+1,centerY-6+dY},bgColor); //left bottom cutout
  gPrim.writePixel(bbuf, centerX-2, centerY+dY, bgColor);  //left iris
  gPrim.writePixel(bbuf, centerX+2, centerY+dY, bgColor);  //right iris
}

//######################
//  Draw Owl
//######################
void GraphicsComposites::drawOwl(FrameBuffer bbuf, int centerX, int centerY,  bool blink, bool squawk) {
  palette c;  // Get color palette

  // head
  gPrim.drawEllipse(bbuf,centerX, centerY+8, 11, 8, 0, c.brown);

  // ears
  gPrim.drawTriangle(bbuf,{centerX-11,centerY+10},{centerX-15,centerY+16},{centerX-3,centerY+10},c.brown); 
  gPrim.drawTriangle(bbuf,{centerX+11,centerY+10},{centerX+15,centerY+16},{centerX+3,centerY+10},c.brown); 

  // eyes
  if(blink) {
    //close eyes
    gPrim.drawEllipse(bbuf,centerX-5, centerY+8,4,3,0,c.brown);
    gPrim.drawEllipse(bbuf,centerX+5, centerY+8,4,3,0,c.brown);
  } else {
    // open eyes
    gPrim.drawEllipse(bbuf,centerX-5, centerY+8,4,3,0,c.white);
    gPrim.drawEllipse(bbuf,centerX+5, centerY+8,4,3,0,c.white);
    gPrim.drawCircle(bbuf,centerX-5,centerY+8, 2, c.black);  // Left iris
    gPrim.drawCircle(bbuf,centerX+5,centerY+8, 2, c.black);  // Right iris
  }
  // eyebrows  (raise them when squawking)
  if(squawk) {
    gPrim.drawArc(bbuf,{centerX-8,centerY+13},{centerX-5,centerY+15},{centerX-2,centerY+13},1,c.yellow);
    gPrim.drawArc(bbuf,{centerX+9,centerY+13},{centerX+6,centerY+15},{centerX+3,centerY+13},1,c.yellow);
  } else {
    gPrim.drawArc(bbuf,{centerX-8,centerY+11},{centerX-5,centerY+13},{centerX-2,centerY+11},1,c.yellow);
    gPrim.drawArc(bbuf,{centerX+9,centerY+11},{centerX+6,centerY+13},{centerX+3,centerY+11},1,c.yellow);
  }

  // beak
  if(squawk) {
    // open beak
    gPrim.drawCircle(bbuf,centerX,centerY+4, 2, c.yellow); 
    gPrim.drawTriangle(bbuf,{centerX-1,centerY+4},{centerX,centerY+3},{centerX+1,centerY+4},c.yellow); 
    gPrim.drawCircle(bbuf,centerX,centerY+4, 1, c.black); 
    gPrim.drawTriangle(bbuf,{centerX-1,centerY+3},{centerX,centerY+2},{centerX+1,centerY+3},c.black); 
  } else {
    // close beak
    gPrim.drawCircle(bbuf,centerX,centerY+4, 2, c.yellow); 
    gPrim.drawTriangle(bbuf,{centerX-1,centerY+4},{centerX,centerY+3},{centerX+1,centerY+4},c.yellow); 
  }

  // Feet
  gPrim.drawTriangle(bbuf,{centerX-4,centerY-9},{centerX-5,centerY-13},{centerX-1,centerY-9},c.yellow); 
  gPrim.drawTriangle(bbuf,{centerX+1,centerY-9},{centerX,centerY-13},{centerX+4,centerY-9},c.yellow); 

  // body
  gPrim.drawEllipse(bbuf,centerX-5, centerY-5, 3, 6, 15, c.brown);
  gPrim.drawEllipse(bbuf,centerX-2,  centerY-5, 3, 6, 15, c.brown);
  gPrim.drawEllipse(bbuf,centerX,    centerY-5, 3, 6, 15, c.brown);
  gPrim.drawEllipse(bbuf,centerX+2,  centerY-5, 3, 6, 15, c.brown);
  gPrim.drawEllipse(bbuf,centerX+5, centerY-5, 3, 6, 15, c.brown);
  gPrim.drawArc(bbuf,{centerX+6,centerY-10},{centerX+3,centerY-3},{centerX+5,centerY+2},1,c.black);

  // wing
  gPrim.drawEllipse(bbuf,centerX-10, centerY-5, 2, 6, -25, c.brown);
}

//######################
//  Draw Pacman 
//######################
void GraphicsComposites::drawPacman(FrameBuffer bbuf,int centerX, int centerY, const struct RGB& bodyColor, const struct RGB& bgColor, bool mouthOpen) {

  // head
  gPrim.drawCircle(bbuf,centerX,centerY,11,bodyColor);

  // Draw Pacman's eye
  gPrim.drawCircle(bbuf,centerX-2,centerY+6,2,bgColor);

  // animate the mouth
  if(mouthOpen) {
    // open mouth is a wedge since we are looking at a side view
    gPrim.drawTriangle(bbuf,{centerX-12,centerY+7}, {centerX-12,centerY-7}, {centerX,centerY}, bgColor);
  } else {
    // Draw the closed mouth (simple horizontal line)
    gPrim.drawRect(bbuf,centerX-11, centerY, centerX-2, centerY,  0,bgColor);
  }
}

//##############################
// Pinecrest elements
//##############################
// campfire
void GraphicsComposites::drawCampfire(FrameBuffer bbuf,int centerX, int centerY) {
  palette c;
  RGB flames[] = {c.red,c.yellow};
  gPrim.drawTriangle(bbuf,{centerX-25,centerY},{centerX-20,random(16,21)},{centerX-15,centerY},flames[random(2)]);
  gPrim.drawTriangle(bbuf,{centerX-15,centerY},{centerX-10,random(16,21)},{centerX-5,centerY},flames[random(2)]);
  gPrim.drawTriangle(bbuf,{centerX-5,centerY},{centerX,random(16,21)},{centerX+5,centerY},flames[random(2)]);
  gPrim.drawTriangle(bbuf,{centerX-20,centerY},{centerX-15,random(16,21)},{centerX-10,centerY},flames[random(2)]);  
  gPrim.drawTriangle(bbuf,{centerX-10,centerY},{centerX-5,random(16,21)},{centerX,centerY},flames[random(2)]);

}

// roasting Stick
void GraphicsComposites::drawRoastingStick(FrameBuffer bbuf,int centerX, int centerY) {
  palette c;
  gPrim.drawLine(bbuf,centerX-9,centerY+3,centerX+10,centerY-4,1,c.gray);  
}

// Pinecrest text
void GraphicsComposites::pinecrestLetters(FrameBuffer bbuf,int startX, int startY) {
  palette c;
  font_7x7 f;
  gPrim.drawLetter(bbuf,f.P,startX ,startY, c.black, c.green);
  gPrim.drawLetter(bbuf,f.I,startX+7 ,startY, c.black, c.green);
  gPrim.drawLetter(bbuf,f.N,startX+14 ,startY, c.black, c.green);
  gPrim.drawLetter(bbuf,f.E,startX+21 ,startY, c.black, c.green);
  gPrim.drawLetter(bbuf,f.C,startX+28 ,startY, c.black, c.green);
  gPrim.drawLetter(bbuf,f.R,startX+35 ,startY, c.black, c.green);
  gPrim.drawLetter(bbuf,f.E,startX+42 ,startY, c.black, c.green);
  gPrim.drawLetter(bbuf,f.S,startX+49 ,startY, c.black, c.green);
  gPrim.drawLetter(bbuf,f.T,startX+56 ,startY, c.black, c.green);

  gPrim.drawLetter(bbuf,f.N2,startX+10 ,startY-10, c.black, c.green);
  gPrim.drawLetter(bbuf,f.N0,startX+17 ,startY-10, c.black, c.green);
  gPrim.drawLetter(bbuf,f.N2,startX+24 ,startY-10, c.black, c.green);
  gPrim.drawLetter(bbuf,f.N6,startX+31 ,startY-10, c.black, c.green);
}
