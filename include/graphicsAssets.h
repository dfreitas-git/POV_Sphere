
#pragma once

#include <Arduino.h>
#include <config.h>
#include <renderer.h>
#include <renderTypes.h>

// Just used so we can have a pointer in the Image structure (see images.h)
// Since the actual callback is inside the graphicsAssets class we can only
// call the method, we can't get the pointer to it, so we wrap the call outside the class.
void imageWrapper(FrameBuffer bbuf);

//#####################
//  Class Definition                         
//#####################
class GraphicsAssets {

public:
  void image(FrameBuffer bbuf);

private:


};
