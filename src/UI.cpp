
#include <UI.h>
#include <motor.h>
#include <Adafruit_SSD1306.h>
#include <ClickEncoder.h>
#include <Ticker.h>

// ######################################
//   Menus and Rotary Encoder for the UI
// #######################################

UI* UI::instance = nullptr;

// UI constructor
UI::UI() :
  oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET),
  encoder(ENC_A, ENC_B, ENC_BTN, 4)
{}

void UI::begin() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed");
    while (true);
  }
  oled.clearDisplay();
  buildMenu();
  instance = this;
  encoderTicker.attach_ms(1, UI::encoderServiceStatic);
}

/* === Encoder ISR ==== */
void UI::encoderServiceStatic() {
  if(UI::instance) UI::instance->encoder.service();
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

// For setting the motorRPM values (motor class controls, UI class just reports and requests changes)
void UI::configureRPM(float value, float minVal, float maxVal) {
    menuRPM.floatValue = value;
    menuRPM.minFloatVal = minVal;
    menuRPM.maxFloatVal = maxVal;
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

/* ===================== Menu Construction ===================== */

MenuItem* currentMenu;

void UI::buildMenu() {

  settingsChildren[0] = &menuBrightness;
  settingsChildren[1] = &menuRPM;

  mainChildren[0] = &menuMotorOnOff;
  mainChildren[1] = &menuScrollOnOff;
  mainChildren[2] = &menuDisplay;
  mainChildren[3] = &menuDemoOnOff;
  mainChildren[4] = &menuSettings;

  menuBrightness = {
    "Brightness",
    MENU_INT_VALUE,
    &menuSettings,
    nullptr, 0,
    nullptr,
    &brightness, 0, 10,
    0, 0, 0,
    nullptr, 0, nullptr
  };

  menuMotorOnOff = {
    "Motor On/Off",
    MENU_ACTION,
    &menuSettings,
    nullptr, 0,
    motorOnOff,
    nullptr, 0, 0,
    0, 0, 0,
    nullptr,0,nullptr
  };

  menuScrollOnOff = {
    "Scroll On/Off",
    MENU_ACTION,
    &menuSettings,
    nullptr, 0,
    scrollOnOff,
    nullptr, 0, 0,
    0, 0, 0,
    nullptr,0,nullptr
  };

  menuDemoOnOff = {
    "Demo On/Off",
    MENU_ACTION,
    &menuSettings,
    nullptr, 0,
    demoOnOff,
    nullptr, 0, 0,
    0, 0, 0,
    nullptr,0,nullptr
  };

  menuRPM = {
    "Motor RPM",
    MENU_FLOAT_VALUE,
    &menuSettings,
    nullptr, 0,
    nullptr,
    nullptr, 0, 0,
    0, 200.0, 400.0,
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
    0, 0, 0,
    imageToDisplay, NUMBER_OF_DISPLAY_FILES, &imageToDisplayIndex
  };

  menuSettings = {
    "Settings",
    MENU_SUBMENU,
    &menuMain,
    settingsChildren,
    2,
    nullptr,
    nullptr, 0, 0,
    0, 0, 0,
    nullptr, 0, nullptr
  };

  menuMain = {
    "Main Menu",
    MENU_SUBMENU,
    nullptr,
    mainChildren,
    5,
    nullptr,
    nullptr, 0, 0,
    0, 0, 0,
    nullptr, 0, nullptr
  };
  currentMenu = &menuMain;
}

/* ====== Display ======== */
void UI::drawMenu() {
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
      oled.print(int(item->floatValue));
    }

    if (item->type == MENU_LIST && showValue) {
      oled.setCursor(60, y);
      oled.print(item->options[*item->optionIndex]);
    }
  }

  oled.display();
}

/* ===================== Input Handling ===================== */
void UI::handleRotation(int delta) {
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
      item->floatValue = constrain(
        item->floatValue + delta,
        item->minFloatVal,
        item->maxFloatVal
      );
      if(! strcmp(item->name, "Motor RPM")) {
        motor.setTargetRPM(item->floatValue);
      }
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

void UI::handleClick() {
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

void UI::handleDoubleClick() {
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
void UI::updateBlink() {
  uint32_t now = millis();
  if (now - lastBlink >= blinkInterval) {
    blinkOn = !blinkOn;
    lastBlink = now;
  }
}

void UI::update() {
    updateBlink();

    // Go check the encoder/switch for user input
    int encoderDelta = encoder.getValue();
    if (encoderDelta != 0) {
      handleRotation(encoderDelta);
    }

    ClickEncoder::Button b = encoder.getButton();
    if (b == ClickEncoder::Clicked) {
      handleClick();
    }
    if (b == ClickEncoder::DoubleClicked) {
      handleDoubleClick();
    }
}
