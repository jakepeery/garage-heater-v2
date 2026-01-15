
#include <DallasTemperature.h>
#include <OneWire.h>
#include <esp_task_wdt.h>
#include <max6675.h>

#include "main.h"

// External reference to USER struct from main.cpp
extern UserSettableData USER;

// Temp Sensor 18B20 Pins - Two room sensors (one on board, one hanging below
// toolbox)
#define TEMP_PIN 27
OneWire oneWire(TEMP_PIN);
DallasTemperature RoomSensor(&oneWire);
#define SensorResolution 11
// options are 9, 10, 11, or 12-bits, which correspond to 0.5°C, 0.25°C,
// 0.125°C, and 0.0625°C, respectively. time for temp polling: 9-0.067s,
// 10-0.140s, 11-0.311s, 12-0.683s time for full loop function to run (as of
// 3/31/23) 9-0.310s, 10-0.407s, 11-0.508s, 12-0.925s

// Thermocouple - Now measuring lower exhaust temps (not top)
int thermoSO = 12;
int thermoCS = 15;
int thermoSCK = 13;
MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);
// MAX6675 thermocouple(MISO, MOSI, SS);

// Sensor validation constants
// DS18B20 sensors stop working around 150°F, garage can get to -20°F
#define MIN_VALID_TEMP_DS18B20 -20.0
#define MAX_VALID_TEMP_DS18B20 150.0
// Thermocouple can handle much higher, but limit to 300°F for sanity check
#define MIN_VALID_TEMP_THERMOCOUPLE -40.0
#define MAX_VALID_TEMP_THERMOCOUPLE 300.0
#define STUCK_VALUE_TIMEOUT 120000  // 2 minutes - reboot if temp unchanged
#define SENSOR_CONSECUTIVE_FAILURES \
  5  // Number of consecutive failures before declaring sensor invalid

// Moving average filter settings
#define TEMP_BUFFER_SIZE 5  // Number of readings to average (5 = ~5 seconds)
#define OUTLIER_THRESHOLD_DS18B20 \
  5.0  // Reject if differs by more than 5°F from average
#define OUTLIER_THRESHOLD_THERMOCOUPLE \
  15.0  // Thermocouple is noisier, allow bigger variance
#define CONSECUTIVE_OUTLIER_ACCEPT \
  3  // Accept trend after 3 consecutive outliers in same direction

// Helper function to calculate moving average with outlier rejection
float calculateFilteredTemp(float newReading, float* buffer, int* bufferIndex,
                            int bufferSize, float outlierThreshold,
                            float lastValid, int sensorId) {
  // Track consecutive outliers per sensor to detect legitimate trends
  static int consecutiveOutlierCount[3] = {0, 0, 0};  // One per sensor
  static float lastOutlierDirection[3] = {0, 0,
                                          0};  // +1 for higher, -1 for lower

  // Calculate current average
  float sum = 0;
  int validCount = 0;
  for (int i = 0; i < bufferSize; i++) {
    if (buffer[i] > -100.0) {  // Skip uninitialized values
      sum += buffer[i];
      validCount++;
    }
  }

  float currentAvg = (validCount > 0) ? (sum / validCount) : newReading;

  // Check if new reading is an outlier
  bool isOutlier = false;
  if (validCount > 0 && abs(newReading - currentAvg) > outlierThreshold) {
    isOutlier = true;

    // Determine direction of outlier
    float direction = (newReading > currentAvg) ? 1.0 : -1.0;

    // Check if this continues the trend from previous outliers
    if (direction == lastOutlierDirection[sensorId]) {
      consecutiveOutlierCount[sensorId]++;
    } else {
      // Direction changed, reset counter
      consecutiveOutlierCount[sensorId] = 1;
      lastOutlierDirection[sensorId] = direction;
    }

    // If we have multiple consecutive outliers in same direction, accept as
    // legitimate trend
    if (consecutiveOutlierCount[sensorId] >= CONSECUTIVE_OUTLIER_ACCEPT) {
      Serial.print("Accepting outlier trend (");
      Serial.print(consecutiveOutlierCount[sensorId]);
      Serial.print(" consecutive): ");
      Serial.print(newReading);
      Serial.println("°F");
      // Reset counter and accept the new reading
      consecutiveOutlierCount[sensorId] = 0;
      // Don't replace newReading - use the actual value
    } else {
      Serial.print("Outlier detected: ");
      Serial.print(newReading);
      Serial.print("°F (avg: ");
      Serial.print(currentAvg);
      Serial.print("°F, diff: ");
      Serial.print(abs(newReading - currentAvg));
      Serial.print("°F, count: ");
      Serial.print(consecutiveOutlierCount[sensorId]);
      Serial.println(") - using previous average");
      // Use the current average instead of the outlier
      newReading = currentAvg;
    }
  } else {
    // Not an outlier, reset consecutive counter
    consecutiveOutlierCount[sensorId] = 0;
  }

  // Add to circular buffer
  buffer[*bufferIndex] = newReading;
  *bufferIndex = (*bufferIndex + 1) % bufferSize;

  // Recalculate average with new value
  sum = 0;
  validCount = 0;
  for (int i = 0; i < bufferSize; i++) {
    if (buffer[i] > -100.0) {
      sum += buffer[i];
      validCount++;
    }
  }

  return (validCount > 0) ? (sum / validCount) : lastValid;
}

void SetupTempSensors() {
  RoomSensor.begin();
  RoomSensor.setResolution(SensorResolution);

  int numberOfMatTempSensors = RoomSensor.getDeviceCount();
  Serial.println("");
  Serial.print("Locating RoomSensor devices...");
  Serial.print("Found ");
  Serial.print(numberOfMatTempSensors, DEC);
  Serial.println(" devices.");
}

void GetTemps(void* pvParameters) {
  // Read Temp Sensors

  Temps* SENSOR_TEMPS;
  SENSOR_TEMPS = (Temps*)pvParameters;

  // Previous values for stuck detection
  float prevRoomTemp = 0;
  float prevExhaustBottom = 0;
  float prevExhaustTop = 0;
  unsigned long lastChangeTime = millis();

  // Failure counters
  int roomTempFailures = 0;
  int exhaustBottomFailures = 0;
  int exhaustTopFailures = 0;

  // Moving average buffers for each sensor
  float onBoardBuffer[TEMP_BUFFER_SIZE];
  float inRoomBuffer[TEMP_BUFFER_SIZE];
  float lowerVentBuffer[TEMP_BUFFER_SIZE];
  int onBoardIndex = 0;
  int inRoomIndex = 0;
  int lowerVentIndex = 0;

  // Initialize buffers with invalid values
  for (int i = 0; i < TEMP_BUFFER_SIZE; i++) {
    onBoardBuffer[i] = -127.0;
    inRoomBuffer[i] = -127.0;
    lowerVentBuffer[i] = -127.0;
  }

  // Fan failure detection variables
  unsigned long gasOnStartTime = 0;
  bool fanCheckActive = false;
#define FAN_CHECK_DELAY 120000  // Wait 2 minutes after gas turns on
#define MIN_VENT_TEMP_INCREASE \
  10.0  // Vent should be at least 10°F hotter than room after 2 min
  float ventTempAtStart = 0;
  float roomTempAtStart = 0;

  while (1) {
    esp_task_wdt_reset();  // Reset watchdog

    // Read temperatures
    RoomSensor.requestTemperatures();

    float roomTempC = RoomSensor.getTempCByIndex(0);
    float roomTemp2C = RoomSensor.getTempCByIndex(1);

    float newRoomTemp = (roomTempC * 9 / 5) + 32;  // DS18B20 #1 - on board
    float newRoomTemp2 =
        (roomTemp2C * 9 / 5) + 32;  // DS18B20 #2 - below toolbox
    float newExhaustTemp =
        thermocouple.readFahrenheit();  // Thermocouple - lower exhaust temp

    // DS18B20 returns -127°C (DEVICE_DISCONNECTED_C) when sensor is
    // disconnected
    bool roomTempConnected = (roomTempC > -100.0);
    bool roomTemp2Connected = (roomTemp2C > -100.0);

    // Validate each sensor reading with appropriate ranges
    bool roomTempValid = roomTempConnected && !isnan(newRoomTemp) &&
                         newRoomTemp > MIN_VALID_TEMP_DS18B20 &&
                         newRoomTemp < MAX_VALID_TEMP_DS18B20;
    bool roomTemp2Valid = roomTemp2Connected && !isnan(newRoomTemp2) &&
                          newRoomTemp2 > MIN_VALID_TEMP_DS18B20 &&
                          newRoomTemp2 < MAX_VALID_TEMP_DS18B20;
    bool exhaustTempValid = !isnan(newExhaustTemp) &&
                            newExhaustTemp > MIN_VALID_TEMP_THERMOCOUPLE &&
                            newExhaustTemp < MAX_VALID_TEMP_THERMOCOUPLE;

    // Apply moving average filter with outlier rejection
    if (roomTempValid) {
      newRoomTemp = calculateFilteredTemp(
          newRoomTemp, onBoardBuffer, &onBoardIndex, TEMP_BUFFER_SIZE,
          OUTLIER_THRESHOLD_DS18B20, SENSOR_TEMPS->ON_BOARD_TEMP, 0);
    }
    if (roomTemp2Valid) {
      newRoomTemp2 = calculateFilteredTemp(
          newRoomTemp2, inRoomBuffer, &inRoomIndex, TEMP_BUFFER_SIZE,
          OUTLIER_THRESHOLD_DS18B20, SENSOR_TEMPS->IN_ROOM_TEMP, 1);
    }
    if (exhaustTempValid) {
      newExhaustTemp = calculateFilteredTemp(
          newExhaustTemp, lowerVentBuffer, &lowerVentIndex, TEMP_BUFFER_SIZE,
          OUTLIER_THRESHOLD_THERMOCOUPLE, SENSOR_TEMPS->LOWER_VENT_TEMP, 2);
    }

    // Track consecutive failures
    roomTempFailures = roomTempValid ? 0 : roomTempFailures + 1;
    exhaustBottomFailures = roomTemp2Valid ? 0 : exhaustBottomFailures + 1;
    exhaustTopFailures = exhaustTempValid ? 0 : exhaustTopFailures + 1;

    // Update connection status
    SENSOR_TEMPS->ON_BOARD_TEMP_CONNECTED = roomTempConnected;
    SENSOR_TEMPS->IN_ROOM_TEMP_CONNECTED = roomTemp2Connected;
    SENSOR_TEMPS->LOWER_VENT_TEMP_CONNECTED =
        exhaustTempValid;  // Thermocouple doesn't have disconnect detection
                           // like DS18B20

    // Update sensor values only if valid
    if (roomTempValid) {
      SENSOR_TEMPS->ON_BOARD_TEMP = newRoomTemp;
    }
    if (roomTemp2Valid) {
      SENSOR_TEMPS->IN_ROOM_TEMP = newRoomTemp2;
    }
    if (exhaustTempValid) {
      SENSOR_TEMPS->LOWER_VENT_TEMP = newExhaustTemp;
    }

    // Check if any temperature has changed (to detect stuck sensors)
    bool anyTempChanged = (abs(newRoomTemp - prevRoomTemp) > 0.2 ||
                           abs(newRoomTemp2 - prevExhaustBottom) > 0.2 ||
                           abs(newExhaustTemp - prevExhaustTop) > 0.5);

    if (anyTempChanged) {
      lastChangeTime = millis();
      prevRoomTemp = newRoomTemp;
      prevExhaustBottom = newRoomTemp2;
      prevExhaustTop = newExhaustTemp;
    }

    // Check for stuck sensors (no change for STUCK_VALUE_TIMEOUT)
    // Only consider "stuck" if at least one sensor is actually connected/valid
    bool anySensorPresent = roomTempConnected || roomTemp2Connected || exhaustTempValid;
    bool stuckSensors = anySensorPresent &&
              (millis() - lastChangeTime) > STUCK_VALUE_TIMEOUT;

    // Fan failure detection - check if lower vent is heating up when gas is on
    // Detect when gas turns on to start the check timer
    if (USER.GAS_ON && !fanCheckActive) {
      fanCheckActive = true;
      gasOnStartTime = millis();
      ventTempAtStart = exhaustTempValid ? newExhaustTemp : 0;
      roomTempAtStart = roomTempValid ? newRoomTemp : 0;
      SENSOR_TEMPS->FAN_FAILURE_DETECTED = false;
      Serial.println("Fan check: Gas turned on, starting 2-minute monitoring");
    }

    // Reset check when gas turns off
    if (!USER.GAS_ON && fanCheckActive) {
      fanCheckActive = false;
      Serial.println("Fan check: Gas turned off, check cancelled");
    }

    // After 2 minutes of gas being on, check if vent temp increased
    if (fanCheckActive && (millis() - gasOnStartTime) > FAN_CHECK_DELAY) {
      if (exhaustTempValid && roomTempValid) {
        float ventTempIncrease = newExhaustTemp - ventTempAtStart;
        float expectedMinTemp = roomTempAtStart + MIN_VENT_TEMP_INCREASE;

        if (newExhaustTemp < expectedMinTemp ||
            ventTempIncrease < MIN_VENT_TEMP_INCREASE) {
          SENSOR_TEMPS->FAN_FAILURE_DETECTED = true;
          Serial.println("*** FAN FAILURE DETECTED ***");
          Serial.print("Lower vent temp: ");
          Serial.print(newExhaustTemp);
          Serial.print("F, Expected at least: ");
          Serial.print(expectedMinTemp);
          Serial.print("F, Increase: ");
          Serial.print(ventTempIncrease);
          Serial.println("F");
          Serial.println("Lower fan may be broken - heater will shut down");
        } else {
          SENSOR_TEMPS->FAN_FAILURE_DETECTED = false;
          Serial.println("Fan check: PASSED - vent heating properly");
        }
      }
      fanCheckActive = false;  // Only check once per gas cycle
    }

    // Determine overall sensor validity
    bool sensorsValid = (roomTempFailures < SENSOR_CONSECUTIVE_FAILURES) &&
                        (exhaustBottomFailures < SENSOR_CONSECUTIVE_FAILURES) &&
                        (exhaustTopFailures < SENSOR_CONSECUTIVE_FAILURES) &&
                        !stuckSensors;

    SENSOR_TEMPS->SENSORS_VALID = sensorsValid;
    SENSOR_TEMPS->LAST_CHANGE_TIME = lastChangeTime;

    // Log sensor issues
    if (!sensorsValid) {
      Serial.println("*** SENSOR VALIDATION FAILED ***");
      if (roomTempFailures >= SENSOR_CONSECUTIVE_FAILURES) {
        if (!roomTempConnected) {
          Serial.println(
              "On Board temperature sensor (DS18B20 #1) DISCONNECTED");
        } else {
          Serial.println(
              "On Board temperature sensor (DS18B20 #1) FAILED - out of range");
        }
      }
      if (exhaustBottomFailures >= SENSOR_CONSECUTIVE_FAILURES) {
        if (!roomTemp2Connected) {
          Serial.println(
              "In Room temperature sensor (DS18B20 #2) DISCONNECTED");
        } else {
          Serial.println(
              "In Room temperature sensor (DS18B20 #2) FAILED - out of "
              "range");
        }
      }
      if (exhaustTopFailures >= SENSOR_CONSECUTIVE_FAILURES) {
        Serial.println(
            "Lower Vent temperature sensor (thermocouple) FAILED - check "
            "wiring");
      }
      if (stuckSensors) {
        Serial.print("Sensors stuck for ");
        Serial.print((millis() - lastChangeTime) / 1000);
        Serial.println(" seconds - REBOOTING IN 10 SECONDS");

        // If stuck for too long, reboot the system
        if ((millis() - lastChangeTime) > (STUCK_VALUE_TIMEOUT + 10000)) {
          Serial.println("REBOOTING NOW DUE TO STUCK SENSORS");
          delay(1000);
          ESP.restart();
        }
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}