
#include <images.h>
#include <graphicsGlobals.h>

// Images to display on the Sphere
const uint8_t NUMBER_OF_DISPLAY_FILES = IMG_COUNT;
const char* imageToDisplay[IMG_COUNT];

// This defines the image functions that will be available to the display
// display menu item in the user UI
const Image *imageTable[IMG_COUNT] = {
    //&testDot,
    //&testL,
    &meridiansWrap,
    &helloWorldWrap,
    &fadeWrap,
    &vFadeWrap,
    &hFadeWrap,
    &fireworksWrap,
    &roadrunWrap,
    &donduckWrap,
    &hBandsWrap,
    &sparkShowerWrap,
    &shootingStarWrap,
    &paintWrap,
    &spiralRWrap,
    &spiralLWrap,
    &spiralDWrap,
    &checkerWrap,
    &flowerWrap,
    &heartsWrap,
    &diamondWrap,
    &pacman1Wrap,
    &pacmanWrap,
    &eyeballWrap,
    &saffronWrap,
    &worldMapWrap,
    &pinecrestWrap,
};
