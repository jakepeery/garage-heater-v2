#include <Arduino.h>
#include <Preferences.h>

#include "LittleFS.h"

// time in ms to reset the encoder press button selection to the default of the
// mode selection not temp selection
// aka wait 5 seconds to go back to selecting on/off/occupied
// Replace with your network credentials

#define reset_time 5000

// Safety temperatures
// Inline duct fan (VIVOSUN 6" 240CFM) max ambient temp is 140°F
// Set exhaust safety limit to 140°F to protect the fan
#define EXHAUST_TEMP_MAX_SAFETY 150
// Must cool below this before resuming gas after exhaust over-temp
#define EXHAUST_TEMP_RESUME 128
// Room sensors should never exceed DS18B20 max (150°F) but add buffer
#define room_temp_max_safety 145

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
  float ON_BOARD_TEMP;    // DS18B20 #1 - on board sensor
  float IN_ROOM_TEMP;     // DS18B20 #2 - in room sensor
  float LOWER_VENT_TEMP;  // Thermocouple - lower vent exhaust temp
  bool SENSORS_VALID = true;
  unsigned long LAST_CHANGE_TIME = 0;

  // Individual sensor status - true if connected and working
  bool ON_BOARD_TEMP_CONNECTED = false;
  bool IN_ROOM_TEMP_CONNECTED = false;
  bool LOWER_VENT_TEMP_CONNECTED = false;

  // Fan failure detection
  bool FAN_FAILURE_DETECTED = false;
};

struct UserSettableData {
  Temps SENSOR_TEMPS;
  char IPAddress[64];
  char WifiMode[64];
  char SYSTEM_STATUS[128] = "Initializing...";
  bool GAS_ON = false;
  bool FAN_ON = false;
  bool EXHAUST_ON = false;
  float LOW_TEMP_SET;   // initialize value from parameter memory
  float HIGH_TEMP_SET;  // initialize value from parameter memory
  int SELECTED_MODE;
  // 1=Unoccupied  2=Heater On  3=Full Off  4=Exhaust Fan  5=Fan and Exhaust On
};
