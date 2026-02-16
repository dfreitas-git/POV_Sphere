
// Header file for all the menu and user interface functions

#pragma once

#include <arduino.h>
#include <stdint.h>
#include <globals.h>
#include <images.h>
#include <Adafruit_GFX.h>

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

extern MenuItem menuRPM;

class Motor;
extern Motor motor;


void motorOnOff(MenuItem*);
void scrollOnOff(MenuItem*);
void buildMenu();
void drawMenu();
void handleRotation(int delta);
void handleClick();
void handleDoubleClick();
void updateBlink();
void encoderService();
