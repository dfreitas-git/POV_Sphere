
#include <graphicsGlobals.h>

//#############################################################################
// Functions for graphics primatives (circles, rectangles, triangles, etc.)
//#############################################################################

//##################################################
//  Write pixel to backBuffer at a given location
//  We use three bytes per pixel for RGB
//##################################################
void GraphicsPrimitives::writePixel(FrameBuffer bbuf, int col, int row, const struct RGB& color) {
  // Be sure they are in-bounds and not negative
  if ((unsigned)col < COLUMNS && (unsigned)row < ROWS) {
      int idx = (row * COLUMNS + col) * 3;
      bbuf[idx]     = color.r;
      bbuf[idx + 1] = color.g;
      bbuf[idx + 2] = color.b;
  } else {
      outOfBoundsPixelCount++;
      Serial.printf("writePixel bounds errors: %d\n",outOfBoundsPixelCount);
  }
}

//##########################################################
//  Draw line with specified two points, thickness and color
//##########################################################
void GraphicsPrimitives::drawLine(FrameBuffer bbuf, int pt0X, int pt0Y, int pt1X, int pt1Y, int thickness, const struct RGB& color) {
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
        writePixel(bbuf, col, row, color);
      }
    }
  }
}
//##########################################################
//  Draw triangle with three vertices specified and color
//##########################################################

// cross product to generate normal vectors for testing if a point lies within the triangle
float GraphicsPrimitives::cross(const Vec2& a, const Vec2& b, const Vec2& c) {
  // Cross product of (b - a) x (c - a)
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// test if point lies within the triangle
bool GraphicsPrimitives::pointInTriangle(const Vec2& p, const Vec2& v1, const Vec2& v2, const Vec2& v3) {
    float d1 = cross(p, v1, v2);
    float d2 = cross(p, v2, v3);
    float d3 = cross(p, v3, v1);

    bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    // Inside or on an edge if all have the same sign (or zero)
    return !(hasNeg && hasPos);
}


void GraphicsPrimitives::drawTriangle(FrameBuffer bbuf, const Vec2& v1, const Vec2& v2, const Vec2& v3, const struct RGB& color) {

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
        writePixel(bbuf,col, row, color);
      }
    }
  }
}

//##########################################################
//  Draw arc with three points, thickness and color
//##########################################################
// Helper functions
// Test if angle-b is between the endpoints when we travel counter-clockwise
bool GraphicsPrimitives::angleBetweenCCW(float a, float b, float c) {
    // true if b is between a and c when sweeping CCW from a to c
    if (a <= c)
        return (b >= a && b <= c);
    else
        return (b >= a || b <= c);
}

// Write the arc's pixels to the framebuffer
void GraphicsPrimitives::writeArcPixels(FrameBuffer bbuf, int cx, int cy, int r, float theta, int thickness, RGB color) {

  int half = thickness / 2;
  float ca = cosf(theta);
  float sa = sinf(theta);
  // Make thickness centered around the original arc
  for (int t = -half; t <= half; t++) {
    int x = cx + (r + t) * ca;
    int y = cy + (r + t) * sa;
    writePixel(bbuf, x, y, color);
  }
}

void GraphicsPrimitives::drawArc(FrameBuffer bbuf,const Vec2& p1, const Vec2& p2, const Vec2& p3, int thickness, const struct RGB& color) {

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
      writeArcPixels(bbuf,cx, cy, r, a, thickness, color);
    }
  } else {
    // Take care of wrapping around 0/360 boundary
    if(theta1 < theta3) {
      theta1 += 2*PI;
    }
    for (float a = theta1; a >= theta3; a -= step) {
      writeArcPixels(bbuf, cx, cy, r, a, thickness, color);
    }
  }
}

//####################################################################
//  Draw filled rect with specified  ll/ur points, rotation and color
//####################################################################
void GraphicsPrimitives::drawRect(FrameBuffer bbuf, int llX, int llY, int urX, int urY, int rotate, const struct RGB& color) {
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
  drawQuad(bbuf,llX, llY, llX, urY, urX, urY, urX, llY, rotate, color);
}

//################################################################
//  Draw filled quadrilateral with specified four points, rotation 
//  angle (in degrees) and color.  Be sure to specify the points 
//  in clockwise order
//################################################################
void GraphicsPrimitives::drawQuad(FrameBuffer bbuf, int pt0X, int pt0Y, int pt1X, int pt1Y, int pt2X, int pt2Y, int pt3X, int pt3Y, int rotate, const struct RGB& color) {

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
  drawTriangle(bbuf,{tp[0],tp[1]}, {tp[2],tp[3]}, {tp[4],tp[5]}, color);
  drawTriangle(bbuf,{tp[0],tp[1]}, {tp[4],tp[5]}, {tp[6],tp[7]}, color);

}

//###########################################################
//  Draw filled diamond to the backbuffer at a given 
// center, with a given minior/major extents and given color
//###########################################################
void GraphicsPrimitives::drawDiamond(FrameBuffer bbuf,int centerX, int centerY, int extentX, int extentY, const struct RGB& color) {
  int plusMinus;
  int leftX = centerX - extentX;
  int leftY = centerY;
  int rightX = centerX + extentX;
  int rightY = centerY;
  int topX = centerX;
  int topY = centerY - extentY;

  // Draw the left half of the diamond with the positive y=mx 
  for (int col = leftX; col <= centerX; col++) {
    plusMinus = (float(leftY-topY)/float(topX-leftX)) * (col-leftX);   // Offset X to run from 0 to centerX
    for(int row = centerY - plusMinus; row <= centerY + plusMinus; row++) {
      writePixel(bbuf,col, row, color);
    }
  }
  // Draw the right half of the diamond with the negative y=mx
  for (int col = topX; col <= rightX; col++) {
    plusMinus = (float(rightY-topY)/float(rightX-topX)) * (rightX-col);   // Offset X to run rightX to 0
    for(int row = centerY - plusMinus; row <= centerY + plusMinus; row++) {
      writePixel(bbuf,col, row, color);
    }
  }
}

//###########################################################
//  Draw filled Ellipse into the backbuffer at a given 
// center, with a given minior/major radius, and given color
// drawEllipse completely written by chatGPT.  Handles rotated 
// ellipses (specify in degrees)
//###########################################################

void GraphicsPrimitives::drawEllipse(FrameBuffer bbuf, int centerX, int centerY, int radiusX, int radiusY, float rotateDeg, const struct RGB& color) {
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
                    writePixel(bbuf,fx, y, color);
                }
            }

            inside = nowInside;
        }

        // Close span at right edge if the second edge was at maxX
        if (inside) {
            for (int fx = spanStartX; fx <= maxX; fx++) {
                writePixel(bbuf,fx, y, color);
            }
        }
    }
}

//####################################################
//  Draw filled circle into the backbuffer at a given 
// center, with a given radius, and given color
//####################################################
void GraphicsPrimitives::drawCircle(FrameBuffer bbuf, int centerX, int centerY, int radius, const struct RGB& color) {
  int plusMinus;

  // circle is symetric so all we need to do is compute +/- Y and fill in the 
  // column pixels between the two points
  for (int col = centerX - radius; col <= centerX + radius; col++) {
    plusMinus = sqrt(pow(radius,2) - pow((col - centerX),2));
    for(int row = centerY - plusMinus; row <= centerY + plusMinus; row++) {
      writePixel(bbuf, col, row, color);
    }
  }
}

//####################################################
//  Draw Letters.  Read fonts from graphicsFunctions.h
//  Letters are 7x5 caps.
//####################################################
void GraphicsPrimitives::drawLetter(FrameBuffer bbuf, uint8_t (*letter)[7], int llX ,int llY, const struct RGB& bgColor, const struct RGB& fgColor){
  for (int col = 0; col < 7; col++) {
    for(int row = 0; row < 7; row++) {
      if(letter[row][col] == 1) {
        writePixel(bbuf,col+llX, llY+6 - row, fgColor);
      } else {
        writePixel(bbuf,col+llX, llY+6 - row, bgColor);
      }
    }
  }
}

//###############################################################
//  Draw spiral with spiral k-factor (-/+ for left/right twist), 
// width,  number of spirals, color.  phase is the offset from 
// col 0 so we can revolve the spirals.
//###############################################################
// Helper function to return column distance across 0-COLUMNS wrap boundary
int GraphicsPrimitives::wrapDist(int a, int b) {
//static inline int wrapDist(int a, int b) {
    int d = a - b;
    if (d >  COLUMNS / 2) d -= COLUMNS;
    if (d < -(COLUMNS / 2)) d += COLUMNS;
    return d;
}

void GraphicsPrimitives::drawSpiral(FrameBuffer bbuf,uint32_t phase, int K, int numSpirals, int thickness, int drawToRow, const struct RGB& color) {
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
          writePixel(bbuf,theta,phi, color); 
          break; // don’t overdraw 
        } 
      } 
    } 
  }
}