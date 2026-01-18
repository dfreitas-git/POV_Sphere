
#pragma Once

#include <arduino.h>
#include <stdint.h>
#include <globals.h>
#include <images.h>
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

extern Adafruit_SSD1306 oled;
extern ClickEncoder encoder;
extern Ticker encoderTicker;

struct MenuItem;
void motorOnOff(MenuItem*);
void scrollOnOff(MenuItem*);
void buildMenu();
void drawMenu();
void handleRotation(int delta);
void handleClick();
void handleDoubleClick();
void updateBlink();