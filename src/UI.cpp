
#include <UI.h>

// ######################################
//   Menus and Rotary Encoder for the UI
// #######################################


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
  float* floatValue;
  float minFloatVal;
  float maxFloatVal;

  /* For option list */
  const char** options;
  uint8_t optionCount;
  uint8_t* optionIndex;
};

// UI objects
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
ClickEncoder encoder(ENC_A, ENC_B, ENC_BTN, 4);
Ticker encoderTicker;


/* === Encoder ISR ==== */
void encoderService() {
  encoder.service();
}


/* ==== Callbacks ========= */
void motorOnOff(MenuItem*) {
  if(motorOnOffFlag == 1) {
    motorOnOffFlag = 0;
     Serial.println("Motor Off");
  } else {
    motorOnOffFlag = 1;
     Serial.println("Motor On");
  }
}

void scrollOnOff(MenuItem*) {
  if(scrollOnOffFlag == 1) {
    scrollOnOffFlag = 0;
     Serial.println("Scroll Off");
  } else {
    scrollOnOffFlag = 1;
     Serial.println("Scroll On");
  }
}

void demoOnOff(MenuItem*) {
  if(demoAll == true) {
    demoAll = false;
     Serial.println("DemoAll Off");
  } else {
    demoAll = true;
     Serial.println("DemoAll On");
  }
}

/* ===================== Menu Declarations ===================== */
MenuItem menuMain;
MenuItem menuSettings;
MenuItem menuBrightness;
MenuItem menuDisplay;
MenuItem menuRPM;
MenuItem menuMotorOnOff;
MenuItem menuScrollOnOff;
MenuItem menuDemoOnOff;


/* ===================== Menu Construction ===================== */
MenuItem* settingsChildren[] = {
  &menuBrightness,
  &menuRPM
};

MenuItem* mainChildren[] = {
  &menuMotorOnOff,
  &menuScrollOnOff,
  &menuDisplay,
  &menuDemoOnOff,
  &menuSettings
};

MenuItem* currentMenu;

void buildMenu() {

  menuMain = {
    "Main Menu",
    MENU_SUBMENU,
    nullptr,
    mainChildren,
    5,
    nullptr,
    nullptr, 0, 0,
    nullptr, 0, 0,
    nullptr, 0, nullptr
  };

  menuSettings = {
    "Settings",
    MENU_SUBMENU,
    &menuMain,
    settingsChildren,
    2,
    nullptr,
    nullptr, 0, 0,
    nullptr, 0, 0,
    nullptr, 0, nullptr
  };

  menuBrightness = {
    "Brightness",
    MENU_INT_VALUE,
    &menuSettings,
    nullptr, 0,
    nullptr,
    &brightness, 0, 10,
    nullptr, 0, 0,
    nullptr, 0, nullptr
  };

  for (int i=0;i<IMG_COUNT;i++) {
    imageToDisplay[i] = imageTable[i]->name;
  }

  menuDisplay = {
    "Display",
    MENU_LIST,
    &menuSettings,
    nullptr, 0,
    nullptr,
    nullptr, 0, 0,
    nullptr, 0, 0,
    imageToDisplay, NUMBER_OF_DISPLAY_FILES, &imageToDisplayIndex
  };

  menuMotorOnOff = {
    "Motor On/Off",
    MENU_ACTION,
    &menuSettings,
    nullptr, 0,
    motorOnOff,
    nullptr, 0, 0,
    nullptr, 0, 0,
    nullptr,0,nullptr
  };

  menuScrollOnOff = {
    "Scroll On/Off",
    MENU_ACTION,
    &menuSettings,
    nullptr, 0,
    scrollOnOff,
    nullptr, 0, 0,
    nullptr, 0, 0,
    nullptr,0,nullptr
  };

  menuDemoOnOff = {
    "Demo On/Off",
    MENU_ACTION,
    &menuSettings,
    nullptr, 0,
    demoOnOff,
    nullptr, 0, 0,
    nullptr, 0, 0,
    nullptr,0,nullptr
  };

  menuRPM = {
    "Motor RPM",
    MENU_FLOAT_VALUE,
    &menuSettings,
    nullptr, 0,
    nullptr,
    nullptr, 0, 0,
    &targetRPM, 200.0, 400.0,
    nullptr, 0, nullptr
  };

  currentMenu = &menuMain;
}

/* ====== Display ======== */
void drawMenu() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  for (uint8_t i = 0; i < currentMenu->childCount; i++) {
    int y = i * 10;

    if (i == currentIndex) {
      oled.setCursor(0, y);
      oled.print(">");
    }
    oled.setCursor(10, y);
    oled.print(currentMenu->children[i]->name);

    MenuItem* item = currentMenu->children[i];
    bool showValue = true;
    if (editingValue && i == currentIndex) {
      showValue = blinkOn;
    }

    if (item->type == MENU_INT_VALUE && showValue) {
      oled.setCursor(90, y);
      oled.print(*item->intValue);
    }

    if (item->type == MENU_FLOAT_VALUE && showValue) {
      oled.setCursor(90, y);
      oled.print(int(*item->floatValue));
    }

    if (item->type == MENU_LIST && showValue) {
      oled.setCursor(60, y);
      oled.print(item->options[*item->optionIndex]);
    }
  }

  oled.display();
}

/* ===================== Input Handling ===================== */
void handleRotation(int delta) {
  MenuItem* item = currentMenu->children[currentIndex];

  if (editingValue) {
    if (item->type == MENU_INT_VALUE) {
      *item->intValue = constrain(
        *item->intValue + delta,
        item->minIntVal,
        item->maxIntVal
      );
    }
    if (item->type == MENU_FLOAT_VALUE) {
      *item->floatValue = constrain(
        *item->floatValue + delta,
        item->minFloatVal,
        item->maxFloatVal
      );
    }

    if (item->type == MENU_LIST) {

      // Wrap the point back to the start if we roll off the end of the list
      int idx = (*item->optionIndex + delta) % item->optionCount;
      if (idx < 0) {
        idx = idx + item->optionCount;
      }
      *item->optionIndex = idx;
    }
  } else {
    int newIndex = (currentIndex + delta) % currentMenu->childCount;
    if (newIndex < 0) {
      newIndex = newIndex + currentMenu->childCount;
    }
    currentIndex = newIndex;
  }
}

void handleClick() {
  MenuItem* item = currentMenu->children[currentIndex];

  if (item->type == MENU_SUBMENU) {
    currentMenu = item;
    currentIndex = 0;
  } else if (item->type == MENU_ACTION) {
    if (item->callback) {
      item->callback(item);
    }
  } else {
    editingValue = !editingValue;
  }
}

void handleDoubleClick() {
  if (editingValue) {
    editingValue = false;
    return;
  }

  if (currentMenu->parent) {
    currentMenu = currentMenu->parent;
    currentIndex = 0;
  }
}

// For blinking the edited menu item
void updateBlink() {
  uint32_t now = millis();
  if (now - lastBlink >= blinkInterval) {
    blinkOn = !blinkOn;
    lastBlink = now;
  }
}