
#pragma once

#include <Arduino.h>
#include <graphicsFunctions.h>

//#####################
//  Class Definition                         
//#####################
class AnimationEngine {

public:
    void update(uint32_t now);
    void render(uint8_t* buffer);

private:

};
