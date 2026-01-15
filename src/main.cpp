

#include "main.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <Update.h>
#include <esp_task_wdt.h>

#include "getTemps.h"
#include "webserver.h"

#define WDT_TIMEOUT 30  // 30 second watchdog timeout

static Preferences storedTemps;
static TimerHandle_t storeTempsOneShotTimer;
static TaskHandle_t ReadTempsTask;

TaskHandle_t MaintainWifiTask;

// overall system power - the heater will not un if false
bool SYSTEM_ON = false;
bool OCCUPIED_ON = false;  // if high heat occupied temp, if false, heat to

float LOW_TEMP_EXISTING;
float HIGH_TEMP_EXISTING;
float MAX_TEMP = 99;
float MIN_TEMP = 32;

float ON_BOARD_TEMP_EXISTING;
float LOWER_VENT_TEMP_EXISTING;
float IN_ROOM_TEMP_EXISTING;

int SELECTED_TEMP = 0;
int SELECTED_TEMP_EXISTING;
unsigned long SELECTED_TEMP_RESET = 0;

// int SELECTED_MODE = 3;  // todo - initialize value from parameter memory
int SELECTED_MODE_EXISTING;
int SELECTED_MODE_MAX = 5;
int SELECTED_MODE_MIN = 1;

UserSettableData USER;

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS \
  0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

class DebouncedButton {
 public:
  bool State = false;
  bool State_Last = false;
  unsigned long LAST_DEBOUNCE_TIME_PRESS = 0;
  unsigned long Delay = 10;
  int PIN;

  boolean buttonDebounce() {
    bool btnState = digitalRead(PIN);
    if (btnState != State_Last) {
      LAST_DEBOUNCE_TIME_PRESS = millis();
    }

    if ((millis() - LAST_DEBOUNCE_TIME_PRESS) > Delay) {
      if (btnState != State) {
        State = btnState;
        Serial.println("GPIO Pressed: " + String(PIN) +
                       " State: " + String(State));
      }
    }
    State_Last = btnState;
    return State;
  }
};

// time in ms the gas should be on before the heater turns on
unsigned long fanDelayTime = 1000;  // in ms

// gas on off times - balanced for continuous warm air with gentle cycling
// Longer on-time reduces valve wear, shorter rest keeps air warm
// These are now stored in preferences and adjustable via web UI
unsigned long gasMaxRunTime = 300000;  // 5 minutes on (default)
unsigned long gasRestTime = 90000;     // 1.5 minutes off (default)
unsigned long gasRestStoppedTime;

// Exhaust temperature management for safety and efficiency
bool exhaustTempLimitTriggered = false;

// Preferences object for storing cycle times
Preferences storedCycleTimes;

// Forward declaration
void stopHeater();

unsigned long gasStartedTime;
bool heaterStarted = false;
void startHeater() {
  // CRITICAL: Never start heater if sensors are invalid
  if (!USER.SENSOR_TEMPS.SENSORS_VALID) {
    Serial.println("*** CANNOT START HEATER - SENSORS INVALID ***");
    stopHeater();
    return;
  }

  // Start gas and exhaust fan
  if (heaterStarted == false) {
    heaterStarted = true;
    USER.GAS_ON = true;
    USER.EXHAUST_ON = true;
    gasStartedTime = millis();
    snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS),
             "Starting heater - gas and exhaust on");
  }

  // When time has elapsed, start fan
  if (heaterStarted && millis() > gasStartedTime + fanDelayTime) {
    USER.FAN_ON = true;
    USER.EXHAUST_ON = true;  // might as well for good luck
    snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS),
             "Heating - gas cycle %d/%d sec",
             (int)((millis() - gasStartedTime) / 1000),
             (int)(gasMaxRunTime / 1000));
  }

  // CRITICAL: If sensors become invalid, immediately shut down
  if (!USER.SENSOR_TEMPS.SENSORS_VALID) {
    Serial.println(
        "*** SENSORS INVALID DURING OPERATION - EMERGENCY SHUTDOWN ***");
    stopHeater();
    return;
  }

  // Check exhaust temp - force shutdown if too hot
    if (USER.GAS_ON == true &&
      USER.SENSOR_TEMPS.LOWER_VENT_TEMP > EXHAUST_TEMP_MAX_SAFETY) {
    Serial.print("*** EXHAUST TOO HOT: ");
    Serial.print(USER.SENSOR_TEMPS.LOWER_VENT_TEMP);
    Serial.println("F - FORCING COOLDOWN ***");
    USER.GAS_ON = false;
    // gasRestStoppedTime = millis();  // Don't set timer for thermal events -
    // use temp-only resume
    exhaustTempLimitTriggered = true;
  }

  // Only let gas stay on for max time
  if (USER.FAN_ON == true && USER.GAS_ON == true &&
      millis() > gasStartedTime + gasMaxRunTime) {
    USER.GAS_ON = false;
    gasRestStoppedTime = millis();
    snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS),
             "Normal cycle - resting for %d sec", (int)(gasRestTime / 1000));
  }

  // Restart gas once conditions are met
  if (USER.FAN_ON == true && USER.GAS_ON == false) {
    bool canResume = false;

    // If thermal limit was triggered, must wait for temperature to drop
    if (exhaustTempLimitTriggered) {
      if (USER.SENSOR_TEMPS.LOWER_VENT_TEMP < EXHAUST_TEMP_RESUME) {
        Serial.print("Exhaust cooled to ");
        Serial.print(USER.SENSOR_TEMPS.LOWER_VENT_TEMP);
        Serial.println("F - OK to resume");
        exhaustTempLimitTriggered = false;
        gasRestStoppedTime = millis() - gasRestTime;  // reset timer
        canResume = true;
      } else {
        // Still too hot - keep waiting
        snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS),
                 "THERMAL WAIT: Exhaust %.1fF - need below %dF to resume",
                 USER.SENSOR_TEMPS.LOWER_VENT_TEMP, EXHAUST_TEMP_RESUME);
        if (millis() - gasRestStoppedTime > 10000) {  // Log every 10 seconds
          static unsigned long lastTempLog = 0;
          if (millis() - lastTempLog > 10000) {
            Serial.print("Waiting for exhaust to cool... currently ");
            Serial.print(USER.SENSOR_TEMPS.LOWER_VENT_TEMP);
            Serial.print("F (need <");
            Serial.print(EXHAUST_TEMP_RESUME);
            Serial.println("F)");
            lastTempLog = millis();
          }
        }
        canResume = false;
      }
    } else {
      // Normal cycle - just wait for time
      unsigned long elapsed = millis() - gasRestStoppedTime;
      unsigned long remaining =
          (elapsed < gasRestTime) ? (gasRestTime - elapsed) / 1000 : 0;
      snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS),
               "Normal rest - %lu sec remaining", remaining);
      canResume = (millis() > gasRestStoppedTime + gasRestTime);
    }

    if (canResume && USER.SENSOR_TEMPS.SENSORS_VALID) {
      gasStartedTime = millis();
      USER.GAS_ON = true;
    } else if (canResume && !USER.SENSOR_TEMPS.SENSORS_VALID) {
      Serial.println("*** CANNOT RESUME GAS - SENSORS INVALID ***");
      stopHeater();
    }
  }
}

// time in ms the fan will keep running after the gas is turned off
unsigned long fanExtendTime = 30000;
unsigned long exhauseFanExtendTime = 60000;

// time the tgas stopped so we can reference it
unsigned long gasStoppedTime;
void stopHeater() {
  // Stop Gas
  // if (!USER.GAS_ON || !USER.FAN_ON)
  if (heaterStarted == true) {
    USER.GAS_ON = false;
    heaterStarted = false;
    gasStoppedTime = millis();
    float targetTemp = OCCUPIED_ON ? USER.HIGH_TEMP_SET : USER.LOW_TEMP_SET;
    snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS),
             "Target reached (%.1fF) - cooling down", targetTemp);
  }

  // run fan and exhaust for 1 minute
  if (heaterStarted == false && millis() > gasStoppedTime + fanExtendTime) {
    // if (millis() > gasStoppedTime + exhauseFanExtendTime) {
    // Serial.println("Here 1");
    if (USER.SENSOR_TEMPS.IN_ROOM_TEMP < 150.0 &&
        USER.SENSOR_TEMPS.LOWER_VENT_TEMP < 200.0) {
      USER.EXHAUST_ON = false;
      Serial.println("Stopping Exhaust Fan");
    } else {
      Serial.println("Stopping... Waiting for exhaust to cool");
    }
    // Serial.println("Here 3");
    USER.FAN_ON = false;
    USER.GAS_ON = false;  // might as well for good luck
    Serial.println("Stopping Fan");
  } else {
    Serial.println("Stopping... Waiting for fan to cool down");
  }
}

void GasOff() {
  digitalWrite(GAS_RELAY, LOW);
  USER.GAS_ON = false;
  if (!exhaustTempLimitTriggered) {
    gasRestStoppedTime = millis();
    // used to force gas to stop the normal rest period
  }
}

bool EvaluateSafetyTemps() {
  // Serial.println("EvaluateSafetyTemps");
  bool isSafe = true;

  // CRITICAL: If sensors are invalid, immediately fail-safe
  if (!USER.SENSOR_TEMPS.SENSORS_VALID) {
    Serial.println("*** SENSORS INVALID - SHUTTING DOWN HEATER ***");
    GasOff();
    return false;
  }

  // CRITICAL: If fan failure detected, immediately fail-safe
  if (USER.SENSOR_TEMPS.FAN_FAILURE_DETECTED) {
    Serial.println("*** FAN FAILURE - SHUTTING DOWN HEATER ***");
    stopHeater();
    return false;
  }

  // Check exhaust temp - must stay below set temp to protect inline duct fan
  if (USER.SENSOR_TEMPS.LOWER_VENT_TEMP > EXHAUST_TEMP_MAX_SAFETY) {
    Serial.print("*** EXHAUST TOO HOT: ");
    Serial.print(USER.SENSOR_TEMPS.LOWER_VENT_TEMP);
    Serial.println("F - SHUTTING DOWN ***");
    exhaustTempLimitTriggered = true;
    GasOff();
    isSafe = false;
  }

  // Check room sensors don't exceed DS18B20 limits
  if (USER.SENSOR_TEMPS.ON_BOARD_TEMP > room_temp_max_safety) {
    Serial.println("*** ON BOARD TEMP TOO HOT - SHUTTING DOWN ***");
    GasOff();
    isSafe = false;
  }

  if (USER.SENSOR_TEMPS.IN_ROOM_TEMP > room_temp_max_safety) {
    Serial.println("*** IN ROOM TEMP TOO HOT - SHUTTING DOWN ***");
    GasOff();
    isSafe = false;
  }

  return isSafe;
}

// heater logic
float SEPARATION_TEMP = 1.0;

void EvaluateCurrentTemps() {
  Serial.println("Evaluate CurrentTems");
  // evaluate system state - ie is the system on, off, or unoccupied
  if (SYSTEM_ON && OCCUPIED_ON && EvaluateSafetyTemps()) {
    // evaluate the temperature readings
    if (USER.SENSOR_TEMPS.IN_ROOM_TEMP < USER.HIGH_TEMP_SET) {
      startHeater();
    } else if (USER.SENSOR_TEMPS.IN_ROOM_TEMP >
               USER.HIGH_TEMP_SET + SEPARATION_TEMP) {
      stopHeater();
    } else {
      // Within target range
      snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS),
               "Maintaining temp: %.1fF (target %.0fF)",
               USER.SENSOR_TEMPS.IN_ROOM_TEMP, USER.HIGH_TEMP_SET);
    }
  } else if (SYSTEM_ON && !OCCUPIED_ON && EvaluateSafetyTemps()) {
    // evaluate the temperature readings
    if (USER.SENSOR_TEMPS.IN_ROOM_TEMP < USER.LOW_TEMP_SET) {
      startHeater();
    } else if (USER.SENSOR_TEMPS.IN_ROOM_TEMP >
               USER.LOW_TEMP_SET + SEPARATION_TEMP) {
      stopHeater();
    } else {
      // Within target range
      snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS),
               "Maintaining temp: %.1fF (target %.0fF)",
               USER.SENSOR_TEMPS.IN_ROOM_TEMP, USER.LOW_TEMP_SET);
    }
  } else {
    // System off or unsafe
    if (!SYSTEM_ON) {
      snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS), "System OFF");
    } else if (!EvaluateSafetyTemps()) {
      snprintf(USER.SYSTEM_STATUS, sizeof(USER.SYSTEM_STATUS),
               "SAFETY SHUTDOWN - temp too high");
    }
  }
}

void LimitVariableRange() {
  // keep temps in the predefined range
  if (USER.HIGH_TEMP_SET < MIN_TEMP) {
    USER.HIGH_TEMP_SET = MIN_TEMP;
  }

  if (USER.LOW_TEMP_SET < MIN_TEMP) {
    USER.LOW_TEMP_SET = MIN_TEMP;
  }

  if (USER.HIGH_TEMP_SET > MAX_TEMP) {
    USER.HIGH_TEMP_SET = MAX_TEMP;
  }

  if (USER.LOW_TEMP_SET > MAX_TEMP) {
    USER.LOW_TEMP_SET = MAX_TEMP;
  }

  if (USER.SELECTED_MODE < SELECTED_MODE_MIN) {
    USER.SELECTED_MODE = SELECTED_MODE_MIN;
  }

  if (USER.SELECTED_MODE > SELECTED_MODE_MAX) {
    USER.SELECTED_MODE = SELECTED_MODE_MAX;
  }
}

void StartupScreen() {
  char stat[64];
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.cp437(true);  // Use full 256 char 'Code Page 437' font

  display.setTextSize(2);
  sprintf(stat, "%s", USER.IPAddress);
  display.println("BOOTING");
  display.println("Please");
  display.println("Wait..");
  display.display();
}

void IpAddressScreen() {
  char stat[64];
  char mode[64];
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.cp437(true);  // Use full 256 char 'Code Page 437' font

  display.setTextSize(2);
  sprintf(stat, "%s", USER.IPAddress);
  sprintf(mode, "%s", USER.WifiMode);
  display.println("Wifi");
  display.setTextSize(1);
  display.print("IP Addr ");
  display.println(stat);
  display.println("");
  display.print("Wifi Mode ");
  display.println(USER.WifiMode);
  display.display();
}

void drawScreen(float tmp1, float tmp2, float tmp3, float highTemp,
                float lowTemp, int mode, int selected) {
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.cp437(true);  // Use full 256 char 'Code Page 437' font

  char stat[64];
  switch (mode) {
    case 1:
      strcpy(stat, "UnOccupied");
      break;
    case 2:
      strcpy(stat, "Heat On");
      break;
    case 3:
      strcpy(stat, "Full Off");
      break;
    case 4:
      strcpy(stat, "Exhaust");
      break;
    case 5:
      strcpy(stat, "Force Fan");
      break;
    case 6:
      sprintf(stat, "%s", USER.IPAddress);
      break;
    default:
      strcpy(stat, "BROKEN");
  }

  display.setTextSize(2);
  if (selected == 0) {
    display.setTextSize(2);
    display.println(stat);

    display.setTextSize(1);
    char highTempPrint[64];
    if (mode == 2) {
      sprintf(highTempPrint, "High %.0f%cF active", highTemp, (char)9);
    } else {
      sprintf(highTempPrint, "High %.0f%cF", highTemp, (char)9);
    }
    display.println(highTempPrint);

    display.setTextSize(1);
    char lowTempPrint[64];
    if (mode == 1) {
      sprintf(lowTempPrint, "Low  %.0f%cF active", lowTemp, (char)9);
    } else {
      sprintf(lowTempPrint, "Low  %.0f%cF", lowTemp, (char)9);
    }
    display.println(lowTempPrint);
  } else if (selected == 1) {
    display.setTextSize(1);
    display.println(stat);

    display.setTextSize(2);
    char highTempPrint[64];
    if (mode == 2) {
      sprintf(highTempPrint, "High %.0f%cF", highTemp, (char)9);
    } else {
      sprintf(highTempPrint, "High %.0f%cF", highTemp, (char)9);
    }
    display.println(highTempPrint);

    display.setTextSize(1);
    char lowTempPrint[64];
    if (mode == 1) {
      sprintf(lowTempPrint, "Low  %.0f%cF active", lowTemp, (char)9);
    } else {
      sprintf(lowTempPrint, "Low  %.0f%cF", lowTemp, (char)9);
    }
    display.println(lowTempPrint);
  } else if (selected == 2) {
    display.setTextSize(1);
    display.println(stat);

    display.setTextSize(1);
    char highTempPrint[64];
    if (mode == 2) {
      sprintf(highTempPrint, "High %.0f%cF active", highTemp, (char)9);
    } else {
      sprintf(highTempPrint, "High %.0f%cF", highTemp, (char)9);
    }
    display.println(highTempPrint);

    display.setTextSize(2);
    char lowTempPrint[64];
    if (mode == 1) {
      sprintf(lowTempPrint, "Low  %.0f%cF", lowTemp, (char)9);
    } else {
      sprintf(lowTempPrint, "Low  %.0f%cF", lowTemp, (char)9);
    }
    display.println(lowTempPrint);
  }

  display.setTextSize(1);
  display.println(F("    Sensor Temps"));

  // If fan failure detected, take over the entire screen
  if (USER.SENSOR_TEMPS.FAN_FAILURE_DETECTED) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.println(F("!! FAN !!"));
    display.println(F("FAILURE"));
    display.setTextSize(1);
    display.println("");
    display.println(F("Lower fan may be"));
    display.println(F("broken!"));
    display.println("");
    display.println(F("System STOPPED"));
    display.display();
    return;  // Skip normal display
  }

  // Display on-board temp or N/A if disconnected
  char buffer[256];
  if (USER.SENSOR_TEMPS.ON_BOARD_TEMP_CONNECTED) {
    if (tmp1 >= 100.0) {
      sprintf(buffer, " On Board   %.1f%cF", tmp1, (char)9);
    } else {
      sprintf(buffer, " On Board    %.1f%cF", tmp1, (char)9);
    }
  } else {
    sprintf(buffer, " On Board    N/A");
  }
  display.println(buffer);

  // Display in-room temp or N/A if disconnected
  char buffer2[256];
  if (USER.SENSOR_TEMPS.IN_ROOM_TEMP_CONNECTED) {
    if (tmp2 >= 100.0) {
      sprintf(buffer2, " In Room    %.1f%cF", tmp2, (char)9);
    } else {
      sprintf(buffer2, " In Room     %.1f%cF", tmp2, (char)9);
    }
  } else {
    sprintf(buffer2, " In Room     N/A");
  }
  display.println(buffer2);

  // Display lower vent temp or N/A if disconnected
  char buffer3[256];
  if (USER.SENSOR_TEMPS.LOWER_VENT_TEMP_CONNECTED) {
    if (tmp3 >= 100.0) {
      sprintf(buffer3, " Lower Vent %.1f%cF", tmp3, (char)9);
    } else {
      sprintf(buffer3, " Lower Vent  %.1f%cF", tmp3, (char)9);
    }
  } else {
    sprintf(buffer3, " Lower Vent  N/A");
  }
  display.println(buffer3);

  display.display();
}

// add interrupts that stop the temperature task when any of the inputs are
// messed with. Then resumes after 1 second of not being re-triggered
// DISABLED - This was causing crashes and is no longer needed
void InterruptForButtonPress() {
  // Interrupt handler removed - using polling instead for stability
  // The task suspension/resume logic was causing memory corruption and reboots
}

void printHardwareInfo() {
  Serial.printf("\n\nChip Cores:         %d\n", ESP.getChipCores());
  Serial.printf("CPU frequency:      %d\n\n", ESP.getCpuFreqMHz());

  Serial.printf("Total heap:         %d\n", ESP.getHeapSize());
  Serial.printf("Free heap:          %d\n", ESP.getFreeHeap());
  Serial.printf("Used heap:          %d\n\n",
                ESP.getHeapSize() - ESP.getFreeHeap());

  Serial.printf("Total PSRAM:        %d\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM:         %d\n\n", ESP.getFreePsram());

  Serial.printf("Total Flash:        %d\n", ESP.getFlashChipSize());
  Serial.printf("Free sketch space:  %d\n", ESP.getFreeSketchSpace());
  Serial.printf("Sketch Size:        %d\n", ESP.getSketchSize());
}

void SaveCurrentTempsCallback(TimerHandle_t xTimer) {
  storedTemps.begin("setTemps", false);
  storedTemps.putFloat("high_temp", USER.HIGH_TEMP_SET);
  storedTemps.putFloat("low_temp", USER.LOW_TEMP_SET);
  storedTemps.putInt("selected_mode", USER.SELECTED_MODE);
  storedTemps.end();
  Serial.println("Stored Values to Flash Memory");
}

//---------------------------------------------------------Setup-------------------------------
DebouncedButton PressBtn;
DebouncedButton UpEncoder;
DebouncedButton DownEncoder;
void setup() {
  Serial.begin(115200);

  // Configure and enable watchdog timer
  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true);  // Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                // Add current thread to WDT watch

  //-----------------------------------------------------------OLED
  // Setup------------------------------------
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);  // Don't proceed, loop forever
  }
  delay(500);
  display.clearDisplay();
  StartupScreen();

  // get stored parameters for temps
  storedTemps.begin("setTemps", false);
  USER.HIGH_TEMP_SET = storedTemps.getFloat("high_temp", 0.0);
  USER.LOW_TEMP_SET = storedTemps.getFloat("low_temp", 0.0);
  USER.SELECTED_MODE = storedTemps.getInt("selected_mode");
  storedTemps.end();

  // Load cycle timing parameters
  storedCycleTimes.begin("cycleTimes", false);
  gasMaxRunTime =
      storedCycleTimes.getULong("gasOnTime", 300000);  // Default 5 min
  gasRestTime =
      storedCycleTimes.getULong("gasRestTime", 90000);  // Default 1.5 min
  storedCycleTimes.end();

  Serial.print("Loaded Gas On Time: ");
  Serial.print(gasMaxRunTime / 60000.0);
  Serial.println(" minutes");
  Serial.print("Loaded Gas Rest Time: ");
  Serial.print(gasRestTime / 60000.0);
  Serial.println(" minutes");

  // initialize temp sensor values incase sensors dont work
  USER.SENSOR_TEMPS.IN_ROOM_TEMP = 20.0;
  USER.SENSOR_TEMPS.LOWER_VENT_TEMP = 20.0;
  USER.SENSOR_TEMPS.ON_BOARD_TEMP = 20.0;
  stopHeater();
  delay(500);
  if (isnan(USER.HIGH_TEMP_SET) || isnan(USER.LOW_TEMP_SET) ||
      isnan(USER.SELECTED_MODE)) {
    Serial.print(
        "\n\nNo parameter for high_temp or low_temp, setting default "
        "values\n\n");
    USER.HIGH_TEMP_SET = 66;
    USER.LOW_TEMP_SET = 38;

    storedTemps.putFloat("high_temp", USER.HIGH_TEMP_SET);
    storedTemps.putFloat("low_temp", USER.LOW_TEMP_SET);
    storedTemps.putInt("selected_mode", USER.SELECTED_MODE);
  } else {
    Serial.print("Read stored values \nhigh_temp: ");
    Serial.println(USER.HIGH_TEMP_SET);
    Serial.print("low_temp: ");
    Serial.println(USER.LOW_TEMP_SET);
    Serial.print("selected_mode: ");
    Serial.println(USER.SELECTED_MODE);
  }

  delay(500);
  printHardwareInfo();

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed (formatted on fail). Web UI may be unavailable.");
  } else {
    Serial.println("LittleFS mounted successfully");
  }

  SetupWebServerWithWifi(&USER);

  // Setup OTA updates
  ArduinoOTA.setHostname("Garage-Heater");
  ArduinoOTA.setPassword("garage123");  // Change this password!
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS (filesystem)
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("OTA Update");
    display.println("Starting...");
    display.display();
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("OTA Update");
    display.println("Complete!");
    display.display();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  // Setup mDNS for easy access via hostname
  if (MDNS.begin("Garage-Heater")) {
    Serial.println("mDNS responder started: http://Garage-Heater.local");
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("Error setting up mDNS responder!");
  }

  IpAddressScreen();
  delay(5000);

  //-----------------------------------------------------Outputs
  //(GPO)-----------------------------------
  pinMode(GAS_RELAY, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(EXHAUST_RELAY, OUTPUT);

  //-----------------------------------------------------Inputs
  //(GPI)-----------------------------------
  pinMode(PRESS, INPUT_PULLDOWN);
  PressBtn.PIN = PRESS;
  PressBtn.Delay = 50;
  // Remove interrupt attachment - polling is more stable for encoders
  // attachInterrupt(PRESS, InterruptForButtonPress, RISING);

  pinMode(UP, INPUT_PULLDOWN);
  UpEncoder.PIN = UP;
  UpEncoder.Delay = 5;
  // Remove interrupt attachment - polling is more stable for encoders
  // attachInterrupt(UP, InterruptForButtonPress, RISING);

  pinMode(DOWN, INPUT_PULLDOWN);
  DownEncoder.PIN = DOWN;
  DownEncoder.Delay = 5;
  // Remove interrupt attachment - polling is more stable for encoders
  // attachInterrupt(DOWN, InterruptForButtonPress, RISING);

  delay(500);

  SetupTempSensors();

  //---------------------------------------------------------------Loops--------------------
  xTaskCreate(
      GetTemps,   /* Task function. */
      "GetTemps", /* String with name of task. */
      4096,       /* Stack size in bytes (reduced from 10000 words = 40KB) */
      (void*)&USER.SENSOR_TEMPS, /* Parameter passed as input of the task */
      2,                         /* Priority of the task. */
      &ReadTempsTask);           /* Task handle. */

  xTaskCreatePinnedToCore(
      MaintainWifi,   /* Task function. */
      "MaintainWifi", /* String with name of task. */
      4096,         /* Stack size in bytes (reduced from 10000 words = 40KB) */
      (void*)&USER, /* Parameter passed as input of the task */
      6,            /* Priority of the task. */
      &MaintainWifiTask,            /* Task handle. */
      CONFIG_ARDUINO_RUNNING_CORE); /* Core to run on. */

  storeTempsOneShotTimer =
      xTimerCreate("Store_Temp_Timer",  // Name of timer - not really used
                   2000 / portTICK_PERIOD_MS,  // Period of timer in ticks
                   pdFALSE,                    // pdTRUE to auto-reload timer
                   (void*)0,                   // Timer ID
                   SaveCurrentTempsCallback);  // the callback function

  // Give timer time to start if needed
  if (storeTempsOneShotTimer == NULL) {
    Serial.println("Could not create timer");
  } else {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Starting timer");
  }
}

bool BTN_PRESSED_STATE = false;
bool UP_ENCODER_STATE = false;

void loop() {
  // Reset watchdog timer in main loop
  esp_task_wdt_reset();

  // Handle OTA updates
  ArduinoOTA.handle();

  // Encoder Press 0= heating function, 1= on temp, 2 = low temp
  bool BTN_Pressed = PressBtn.buttonDebounce();
  if (BTN_Pressed != BTN_PRESSED_STATE && BTN_Pressed && SELECTED_TEMP == 0) {
    SELECTED_TEMP = 1;
    SELECTED_TEMP_RESET = millis() + reset_time;
  } else if (BTN_Pressed != BTN_PRESSED_STATE && BTN_Pressed &&
             SELECTED_TEMP == 1) {
    SELECTED_TEMP = 2;
    SELECTED_TEMP_RESET = millis() + reset_time;
  } else if (BTN_Pressed != BTN_PRESSED_STATE && BTN_Pressed &&
             SELECTED_TEMP == 2) {
    SELECTED_TEMP = 0;
    SELECTED_TEMP_RESET = millis() + reset_time;
  }

  BTN_PRESSED_STATE = BTN_Pressed;

  // reset to function select
  if (millis() > SELECTED_TEMP_RESET) {
    SELECTED_TEMP = 0;
  }

  // Encoder Direction
  bool triggered_up = digitalRead(UP);      // UpEncoder.buttonDebounce(); //
  bool triggered_down = digitalRead(DOWN);  // DownEncoder.buttonDebounce(); //

  // if (triggered_up) {
  //   Serial.println("triggered_up Pressed");
  // }

  // if (triggered_down) {
  //   Serial.println("triggered_down Pressed");
  // }

  if ((triggered_up != UP_ENCODER_STATE) && triggered_up) {
    if (triggered_down != triggered_up) {
      SELECTED_TEMP_RESET = millis() + reset_time;
      Serial.println("up");
      if (SELECTED_TEMP == 1) {
        USER.HIGH_TEMP_SET = USER.HIGH_TEMP_SET + 1;
        Serial.println("USER.HIGH_TEMP_SET = " + String(USER.HIGH_TEMP_SET));
      } else if (SELECTED_TEMP == 2) {
        USER.LOW_TEMP_SET = USER.LOW_TEMP_SET + 1;
        Serial.println("USER.LOW_TEMP_SET = " + String(USER.LOW_TEMP_SET));
      } else if (SELECTED_TEMP == 0) {
        USER.SELECTED_MODE = USER.SELECTED_MODE + 1;
        Serial.println("SELECTED_MODE = " + String(USER.SELECTED_MODE));
      }
    } else {
      SELECTED_TEMP_RESET = millis() + reset_time;
      Serial.println("down");
      if (SELECTED_TEMP == 1) {
        USER.HIGH_TEMP_SET = USER.HIGH_TEMP_SET - 1;
        Serial.println("USER.HIGH_TEMP_SET = " + String(USER.HIGH_TEMP_SET));
      } else if (SELECTED_TEMP == 2) {
        USER.LOW_TEMP_SET = USER.LOW_TEMP_SET - 1;
        Serial.println("USER.LOW_TEMP_SET = " + String(USER.LOW_TEMP_SET));
      } else if (SELECTED_TEMP == 0) {
        USER.SELECTED_MODE = USER.SELECTED_MODE - 1;
        Serial.println("SELECTED_MODE = " + String(USER.SELECTED_MODE));
      }
    }
  }
  UP_ENCODER_STATE = triggered_up;

  LimitVariableRange();

  // logic for the different modes
  if (USER.SELECTED_MODE == 1) {  // Unoccupied
    SYSTEM_ON = true;
    OCCUPIED_ON = false;
  } else if (USER.SELECTED_MODE == 2) {  // Heater On
    SYSTEM_ON = true;
    OCCUPIED_ON = true;
  } else if (USER.SELECTED_MODE == 3) {  // Full Off
    SYSTEM_ON = false;
    OCCUPIED_ON = false;
    USER.EXHAUST_ON = false;
    USER.FAN_ON = false;
    USER.GAS_ON = false;
    heaterStarted = false;
    // stopHeater();
  } else if (USER.SELECTED_MODE == 4) {  // Exhaust Only
    SYSTEM_ON = false;
    OCCUPIED_ON = false;
    USER.EXHAUST_ON = true;
    USER.FAN_ON = false;
    USER.GAS_ON = false;
  } else if (USER.SELECTED_MODE == 5) {  // Force Fan
    SYSTEM_ON = false;
    OCCUPIED_ON = false;
    USER.EXHAUST_ON = true;
    USER.FAN_ON = true;
    USER.GAS_ON = false;
  }

  if (USER.EXHAUST_ON) {
    digitalWrite(EXHAUST_RELAY, HIGH);
  } else {
    digitalWrite(EXHAUST_RELAY, LOW);
  }

  if (USER.FAN_ON) {
    digitalWrite(FAN_RELAY, HIGH);
  } else {
    digitalWrite(FAN_RELAY, LOW);
  }

  if (USER.GAS_ON) {
    digitalWrite(GAS_RELAY, HIGH);
  } else {
    digitalWrite(GAS_RELAY, LOW);
  }

  // Update screen if values change
  if (LOW_TEMP_EXISTING != USER.LOW_TEMP_SET ||
      HIGH_TEMP_EXISTING != USER.HIGH_TEMP_SET ||
      SELECTED_TEMP != SELECTED_TEMP_EXISTING ||
      ON_BOARD_TEMP_EXISTING != USER.SENSOR_TEMPS.ON_BOARD_TEMP ||
      LOWER_VENT_TEMP_EXISTING != USER.SENSOR_TEMPS.LOWER_VENT_TEMP ||
      IN_ROOM_TEMP_EXISTING != USER.SENSOR_TEMPS.IN_ROOM_TEMP ||
      USER.SELECTED_MODE != SELECTED_MODE_EXISTING) {
    //
    // Save current set temps to flash storage
    // only if the set temps changed
    if (LOW_TEMP_EXISTING != USER.LOW_TEMP_SET ||
        HIGH_TEMP_EXISTING != USER.HIGH_TEMP_SET ||
        USER.SELECTED_MODE != SELECTED_MODE_EXISTING) {
      xTimerStart(storeTempsOneShotTimer, 100);
    }

    // Set all existing values so we can get into this if statement the next
    // cycle
    LOW_TEMP_EXISTING = USER.LOW_TEMP_SET;
    HIGH_TEMP_EXISTING = USER.HIGH_TEMP_SET;
    SELECTED_TEMP_EXISTING = SELECTED_TEMP;
    ON_BOARD_TEMP_EXISTING = USER.SENSOR_TEMPS.ON_BOARD_TEMP;
    LOWER_VENT_TEMP_EXISTING = USER.SENSOR_TEMPS.LOWER_VENT_TEMP;
    IN_ROOM_TEMP_EXISTING = USER.SENSOR_TEMPS.IN_ROOM_TEMP;
    SELECTED_MODE_EXISTING = USER.SELECTED_MODE;

    // write to the OLED
    drawScreen(USER.SENSOR_TEMPS.ON_BOARD_TEMP, USER.SENSOR_TEMPS.IN_ROOM_TEMP,
               USER.SENSOR_TEMPS.LOWER_VENT_TEMP, USER.HIGH_TEMP_SET,
               USER.LOW_TEMP_SET, USER.SELECTED_MODE, SELECTED_TEMP);
    EvaluateCurrentTemps();
  }

  // Removed problematic task suspension/resume logic that was causing crashes
  // Tasks now run continuously without interruption

  // Small delay to prevent watchdog issues
  delay(10);
}