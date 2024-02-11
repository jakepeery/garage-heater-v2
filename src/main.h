#include <Arduino.h>

#include "SPIFFS.h"

// time in ms to reset the encoder press button selection to the default of the
// mode selection not temp selection
// aka wait 5 seconds to go back to selecting on/off/occupied
// Replace with your network credentials

#define reset_time 5000

// safety temperatures
#define exhaust_top_max_temp_safety 250
#define exhaust_bottom_max_temp_safety 200

// relay pins - GPO
#define GAS_RELAY 18
#define FAN_RELAY 19
#define EXHAUST_RELAY 23

// encoder pins - GPI
#define PRESS 25
#define UP 32
#define DOWN 33

// OLED Piuns
#define OLED_SCL 22
#define OLED_SDA 21

struct Temps {
  float ROOM_TEMP;
  float EXHAUST_BOTTOM_TEMP;
  float EXHAUST_TOP_TEMP;
};

struct UserSettableData {
  Temps SENSOR_TEMPS;
  char IPAddress[64];
  char WifiMode[64];
  bool GAS_ON = false;
  bool FAN_ON = false;
  bool EXHAUST_ON = false;
  float LOW_TEMP_SET;   // initialize value from parameter memory
  float HIGH_TEMP_SET;  // initialize value from parameter memory
  int SELECTED_MODE;
  // 1=Unoccupied  2=Heater On  3=Full Off  4=Exhaust Fan  5=Fan and Exhaust On
};
