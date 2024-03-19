
#include <DallasTemperature.h>
#include <OneWire.h>
#include <max6675.h>

#include "main.h"

// Temp Sensor 18B20 Pins
#define TEMP_PIN 27
OneWire oneWire(TEMP_PIN);
DallasTemperature RoomSensor(&oneWire);
#define SensorResolution 11
// options are 9, 10, 11, or 12-bits, which correspond to 0.5°C, 0.25°C,
// 0.125°C, and 0.0625°C, respectively. time for temp polling: 9-0.067s,
// 10-0.140s, 11-0.311s, 12-0.683s time for full loop function to run (as of
// 3/31/23) 9-0.310s, 10-0.407s, 11-0.508s, 12-0.925s

// Thermocouple
int thermoSO = 12;
int thermoCS = 15;
int thermoSCK = 13;
MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);
// MAX6675 thermocouple(MISO, MOSI, SS);

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

void GetTemps(void *pvParameters) {
  // Read Temp Sensors

  Temps *SENSOR_TEMPS;
  SENSOR_TEMPS = (Temps *)pvParameters;
  while (1) {
    // Serial.println("Requesting temperatures...");
    RoomSensor.requestTemperatures();

    (*SENSOR_TEMPS).ROOM_TEMP = (RoomSensor.getTempCByIndex(0) * 9 / 5) + 32;
    SENSOR_TEMPS->EXHAUST_BOTTOM_TEMP =
        (RoomSensor.getTempCByIndex(1) * 9 / 5) + 32;
    // Start Attempted fix+
    float tempTop = thermocouple.readFahrenheit();
    // if (tempTop == NAN) {
    //   SENSOR_TEMPS->EXHAUST_TOP_TEMP = SENSOR_TEMPS->EXHAUST_TOP_TEMP;
    // } else {
    //   SENSOR_TEMPS->EXHAUST_TOP_TEMP = thermocouple.readFahrenheit();
    // }

    if (tempTop != NAN) {
      SENSOR_TEMPS->EXHAUST_TOP_TEMP = thermocouple.readFahrenheit();
    }

    // end attempted fix
    // SENSOR_TEMPS->EXHAUST_TOP_TEMP = thermocouple.readFahrenheit();
    vTaskDelay(1000);
  }
}