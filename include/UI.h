
// Header file for all the menu and user interface functions

#pragma once

#include <arduino.h>
#include <stdint.h>
#include <globals.h>
#include <images.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ClickEncoder.h>
#include <Ticker.h>


enum MenuItemType {
  MENU_SUBMENU,
  MENU_ACTION,
  MENU_INT_VALUE,
  MENU_FLOAT_VALUE,
  MENU_LIST
};

typedef void (*ActionCallback)(MenuItem*);

struct MenuItem {
  const char* name;
  MenuItemType type;
  MenuItem* parent;

  /* For submenu */
  MenuItem** children;
  uint8_t childCount;

  /* For callbacks */
  ActionCallback callback;

  /* For int value */
  int* intValue;
  int minIntVal;
  int maxIntVal;

  /* For float value */
  float floatValue;
  float minFloatVal;
  float maxFloatVal;

  /* For option list */
  const char** options;
  uint8_t optionCount;
  uint8_t* optionIndex;
};

class Motor;
extern Motor motor;

void motorOnOff(MenuItem*);
void scrollOnOff(MenuItem*);
void demoOnOff(MenuItem*);

class UI {
public:
    UI();
    void begin();
    void update();
    void drawMenu();

    // Rotary switch
    void handleRotation(int delta);
    void handleClick();
    void handleDoubleClick();

    // motor menu access
    void configureRPM(float value, float minVal, float maxVal);

    // Which image to display
    void setImageToDisplayIndex(uint8_t index);
    uint8_t getImageToDisplayIndex();
    void setPreviousImageToDisplayIndex(uint8_t index);
    uint8_t getPreviousImageToDisplayIndex();

    bool demoAllMode();
    bool demoAll = false; // Demo mode - When true, rotate through all the display animations

private:
    void buildMenu();
    void updateBlink();

    // Need these static variables to avoid the "this" pointer 
    // since the tickerEncoder can't deal with that
    static UI* instance;
    static void encoderServiceStatic();

    // ===== Hardware =====
    Adafruit_SSD1306 oled;
    ClickEncoder encoder;
    Ticker encoderTicker;

    // ===== Menu State =====
    MenuItem* currentMenu;
    int currentIndex = 0;
    bool editingValue = false;

    // Blink state
    bool blinkOn = true;
    uint32_t lastBlink = 0;
    const uint32_t blinkInterval = 500;

    // Image to display
    uint8_t imageToDisplayIndex = 0;
    uint8_t previousImageToDisplayIndex = 0;

    // ===== Menu Objects =====
    MenuItem menuMain;
    MenuItem menuSettings;
    MenuItem menuBrightness;
    MenuItem menuDisplay;
    MenuItem menuRPM;
    MenuItem menuMotorOnOff;
    MenuItem menuScrollOnOff;
    MenuItem menuDemoOnOff;

    MenuItem* settingsChildren[4];
    MenuItem* mainChildren[5];

};
