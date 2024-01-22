#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <max6675.h>

// #define MOSI 13
// #define MISO 12
// #define SCK 14
// #define SS 15

int thermoSO = 12;
int thermoCS = 15;
int thermoSCK = 13;

// int thermoDO = 19;
// int thermoCS = 23;
// int thermoCLK = 5;


MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);
//MAX6675 thermocouple(MISO, MOSI, SS);

// relay pins
const int GAS_RELAY = 18;
const int FAN_RELAY = 19;
const int EXHAUST_RELAY = 23;

// switch pins
const int PRESS = 25;
const int UP = 32;
const int DOWN = 33;

//OLED Piuns
const int OLED_SCL = 22;
const int OLED_SDA = 21;

//Temp Sensor 18B20 Pins
const int TEMP1 = 4;
const int TEMP2 = 35;

bool SYSTEM_ON = false;   // overall system power - the heater will not un if false
bool OCCUPIED_ON = false; // if high heat occupied temp, if false, heat to unoccupied temp - aka do not freeze

bool UNDER_LOW_TEMP = false;
bool UNDER_HIGH_TEMP = false;

bool HEATER_ON = false;
bool FAN_ON = false;
bool EXHAUST_ON = false;

float LOW_TEMP_SET = 40;
float HIGH_TEMP_SET = 68;
float LOW_TEMP_EXISTING;
float HIGH_TEMP_EXISTING;
float MAX_TEMP = 80;
float MIN_TEMP = 32;

float ROOM_TEMP;
float EXHAUST_TOP_TEMP;
float EXHAUST_BOTTOM_TEMP;

float ROOM_TEMP_EXISTING;
float EXHAUST_TOP_TEMP_EXISTING;
float EXHAUST_BOTTOM_TEMP_EXISTING;

int SELECTED_TEMP = 1;
int SELECTED_TEMP_EXISTING;


int globalIntVar = 5;
TaskHandle_t Task1;
unsigned long RESTART_TIME = 0;
bool Task1_Running = false;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



OneWire oneWire(TEMP1);
DallasTemperature RoomSensor(&oneWire);



class DebouncedButton {
  public:
    bool State = false;
    bool State_Last = false;
    unsigned long LAST_DEBOUNCE_TIME_PRESS = 0;
    unsigned long Delay = 10;
    int PIN;

    boolean buttonDebounce() {
      bool btnState = digitalRead(PIN);
      if (btnState != State_Last)
      {
        LAST_DEBOUNCE_TIME_PRESS = millis();
        //Serial.println("GPIO Pressed: " + String(PIN));
      }

      //Serial.println("(millis() - LAST_DEBOUNCE_TIME_PRESS) Value: " + String((millis() - LAST_DEBOUNCE_TIME_PRESS)) + " Last BebounceTime: " + String(LAST_DEBOUNCE_TIME_PRESS));
      if ((millis() - LAST_DEBOUNCE_TIME_PRESS) > Delay){
        if (btnState != State){
          State = btnState;
          Serial.println("GPIO Pressed: " + String(PIN) + " State: " + String(State));
        }
      }
      State_Last = btnState;
      return State;
    }
};



void startHeater()
{
  digitalWrite(GAS_RELAY, HIGH);
  digitalWrite(FAN_RELAY, HIGH);
  digitalWrite(EXHAUST_RELAY, HIGH);
}

void stopHeater()
{
  digitalWrite(EXHAUST_RELAY, LOW);
  digitalWrite(FAN_RELAY, LOW);
  digitalWrite(GAS_RELAY, LOW);
}



void drawScreen(float tmp1, float tmp2, float tmp3, float highTemp, float lowTemp, int displayType, int selected) {
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // display.setTextSize(2);
  // display.println(F("High   Low"));
  display.setTextSize(2);

  if (selected == 1){
    char highTempPrint[64];
    sprintf(highTempPrint, "*High %.0f%cF", highTemp, (char)9);
    display.println(highTempPrint);
    //display.print("");

    char lowTempPrint[64];
    sprintf(lowTempPrint, " Low  %.0f%cF", lowTemp, (char)9);
    display.println(lowTempPrint);

  } else if (selected == 2){
    char highTempPrint[64];
    sprintf(highTempPrint, " High %.0f%cF", highTemp, (char)9);
    display.println(highTempPrint);
    //display.print("");

    char lowTempPrint[64];
    sprintf(lowTempPrint, "*Low  %.0f%cF", lowTemp, (char)9);
    display.println(lowTempPrint);
  }else {
    display.println(F("NO SEL"));
  }

  
  display.setTextSize(1);
  display.println(F("Sensor Temps"));

  //handles two vs 3 digit temps for left alignment
  char buffer[256];
  if (tmp1 >= 100.0) {
    sprintf(buffer, " %.1f%cF    %.1f%cF", tmp1, (char)9, tmp2, (char)9);
  } else {
    sprintf(buffer, " %.1f%cF%s    %.1f%cF", tmp1, (char)9, " ", tmp2, (char)9);
  }
  display.println(buffer);

  //handles two vs 3 digit temps for left alignment
  char buffer2[256];
  if (tmp3 >= 100.0) {
    sprintf(buffer2, " %.1f%cF    %.1f%cF", tmp3, (char)9, 99.9, (char)9);
  } else {
    sprintf(buffer2, " %.1f%cF%s    %.1f%cF", tmp3, (char)9, " ", 99.9, (char)9);
  }
  display.println(buffer2);

  //State of the system (heating, cooling, standby, etc)

  char stat[64];
  switch(displayType) {
  case 1:
    strcpy(stat, "Heating");
    //char str[64] = "Heating";
    break;
  case 2:
    strcpy(stat, "Cooling");
    //char str[64] = "Cooling";
    break;
  case 3:
    strcpy(stat, "Dual Fan Cooling");
    //char str[64] = "Dual Fan Cooling";
    break;
  case 4:
    strcpy(stat, "Cold Shutoff");
    //char str[64] = "Cold Shutoff";
    break;
  case 5:
    strcpy(stat, "Over Temp Shutoff");
    //char str[64] = "Over Temp Shutoff";
    break;
  case 6:
    strcpy(stat, "Sensor Error");
    //char str[64] = "Sensor Error";
    break;
  default:
    strcpy(stat, "standby");
    //char str[64] = "standby";
  }
  display.println(stat);
  
  display.display();
}




void GetTemps(void* pvParameters) {
  //Read Temp Sensors
  for(;;){
  Serial.println("Requesting temperatures...");
  
  Serial.println(millis());

  RoomSensor.requestTemperatures(); 
  Serial.println("Got temperatures...");
  
  Serial.println(millis());

  ROOM_TEMP = (RoomSensor.getTempCByIndex(0) * 9 / 5) + 32;
  EXHAUST_BOTTOM_TEMP = (RoomSensor.getTempCByIndex(1) * 9 / 5) + 32;
  Serial.print("Room: ");
  Serial.print(ROOM_TEMP);
  Serial.print("     Fan: ");
  Serial.println(EXHAUST_BOTTOM_TEMP);
  
  Serial.println(millis());
  
  Serial.print("C = "); 
  Serial.println(thermocouple.readCelsius());
  Serial.print("F = ");
  Serial.println(thermocouple.readFahrenheit());
  Serial.println(millis());
  vTaskDelay(1000);
  }
}


void InterruptForButtonPress(){
  if (Task1_Running) {
    //Serial.println("Interrupt Task");
    vTaskSuspend(Task1);
    RESTART_TIME = millis() + 100;
    Task1_Running = false;
  }
}










DebouncedButton PressBtn;
DebouncedButton UpEncoder;
DebouncedButton DownEncoder;
void setup(){
  Serial.begin(9600);


  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS); 

  //-----------------------------------------------------------OLED------------------------------------
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Clear the buffer
  display.clearDisplay();

  //-----------------------------------------------------Outputs (GPO)-----------------------------------
  pinMode(GAS_RELAY, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(EXHAUST_RELAY, OUTPUT);


  //-----------------------------------------------------Inputs (GPO)-----------------------------------
  pinMode(PRESS, INPUT_PULLDOWN);
  PressBtn.PIN = PRESS;
  PressBtn.Delay = 50;
  attachInterrupt(PRESS, InterruptForButtonPress, RISING);

  pinMode(UP, INPUT_PULLDOWN);
  UpEncoder.PIN = UP;
  UpEncoder.Delay = 5;
  attachInterrupt(UP, InterruptForButtonPress, RISING);

  pinMode(DOWN, INPUT_PULLDOWN);
  DownEncoder.PIN = DOWN;
  DownEncoder.Delay = 5;
  attachInterrupt(DOWN, InterruptForButtonPress, RISING);

  

  //-----------------------------------------------------Temperature Sensors---------------------------
  int numberOfMatTempSensors = RoomSensor.getDeviceCount();
  Serial.println("");
  Serial.print("Locating RoomSensor devices...");
  Serial.print("Found ");
  Serial.print(numberOfMatTempSensors, DEC);
  Serial.println(" devices.");

  
  const int SensorResolution = 11;
  RoomSensor.begin();
  RoomSensor.setResolution(SensorResolution);  
  //options are 9, 10, 11, or 12-bits, which correspond to 0.5°C, 0.25°C, 0.125°C, and 0.0625°C, respectively.
  //time for temp polling: 9-0.067s, 10-0.140s, 11-0.311s, 12-0.683s
  //time for full loop function to run (as of 3/31/23) 9-0.310s, 10-0.407s, 11-0.508s, 12-0.925s


  xTaskCreatePinnedToCore(
    GetTemps,             /* Task function. */
    "GetTemps",           /* String with name of task. */
    10000,                     /* Stack size in words. */
    (void*)&globalIntVar,      /* Parameter passed as input of the task */
    5,                         /* Priority of the task. */
    &Task1,                 /* Task handle. */
    0);                     /* Core. */
}


bool BTN_PRESSED_STATE = false;
bool UP_ENCODER_STATE = false;
bool DOWN_ENCODER_STATE = false;
bool SHOULD_MOVE = true;


//add interrupts that stop the temperature task when any of the inputs are messed with.  
//Then resumes after 1 second of not being re-triggered


void loop()
{

  //Encoder Press
  bool BTN_Pressed = PressBtn.buttonDebounce();
  if (BTN_Pressed != BTN_PRESSED_STATE && BTN_Pressed && SELECTED_TEMP == 1)
  {
    SELECTED_TEMP = 2;
    Serial.println("SELECTED_TEMP = 2");
  }
  else if (BTN_Pressed != BTN_PRESSED_STATE && BTN_Pressed && SELECTED_TEMP == 2)
  {
    SELECTED_TEMP = 1;
    Serial.println("SELECTED_TEMP = 1");
  }
  BTN_PRESSED_STATE = BTN_Pressed;


  //Encoder Direction
  bool triggered_up = digitalRead(UP); // UpEncoder.buttonDebounce(); //
  bool triggered_down = digitalRead(DOWN);  //DownEncoder.buttonDebounce(); //


  if ((triggered_up != UP_ENCODER_STATE) && triggered_up) {
    if (triggered_down != triggered_up){
      Serial.println("up");
      if (SELECTED_TEMP == 1) {
        HIGH_TEMP_SET = HIGH_TEMP_SET + 1;
        Serial.println("HIGH_TEMP_SET = " + String(HIGH_TEMP_SET));
      } else if (SELECTED_TEMP == 2) {
        LOW_TEMP_SET = LOW_TEMP_SET + 1;
        Serial.println("LOW_TEMP_SET = " + String(LOW_TEMP_SET));
      }
    }else {
      Serial.println("down");
      if (SELECTED_TEMP == 1) {
        HIGH_TEMP_SET = HIGH_TEMP_SET - 1;
        Serial.println("HIGH_TEMP_SET = " + String(HIGH_TEMP_SET));
      } else if (SELECTED_TEMP == 2) {
        LOW_TEMP_SET = LOW_TEMP_SET - 1;
        Serial.println("LOW_TEMP_SET = " + String(LOW_TEMP_SET));
      }
    }
  }
  UP_ENCODER_STATE = triggered_up;
 

  //keep temps in the predefined range
  if (HIGH_TEMP_SET < MIN_TEMP)
  {
    HIGH_TEMP_SET = MIN_TEMP;
  }

  if (LOW_TEMP_SET < MIN_TEMP)
  {
    LOW_TEMP_SET = MIN_TEMP;
  }

  if (HIGH_TEMP_SET > MAX_TEMP)
  {
    HIGH_TEMP_SET = MAX_TEMP;
  }

  if (LOW_TEMP_SET > MAX_TEMP)
  {
    LOW_TEMP_SET = MAX_TEMP;
  }

  if (SELECTED_TEMP == 1) {
    startHeater();
  } else if (SELECTED_TEMP == 2){
    stopHeater();
  }



  //Update screen if values change
  if (LOW_TEMP_EXISTING != LOW_TEMP_SET || HIGH_TEMP_EXISTING != HIGH_TEMP_SET ||SELECTED_TEMP != SELECTED_TEMP_EXISTING
  || ROOM_TEMP_EXISTING != ROOM_TEMP|| EXHAUST_TOP_TEMP_EXISTING != EXHAUST_TOP_TEMP|| EXHAUST_BOTTOM_TEMP_EXISTING != EXHAUST_BOTTOM_TEMP) {
    LOW_TEMP_EXISTING = LOW_TEMP_SET;
    HIGH_TEMP_EXISTING = HIGH_TEMP_SET;
    SELECTED_TEMP_EXISTING = SELECTED_TEMP;
    ROOM_TEMP_EXISTING = ROOM_TEMP;
    EXHAUST_TOP_TEMP_EXISTING = EXHAUST_TOP_TEMP;
    EXHAUST_BOTTOM_TEMP_EXISTING = EXHAUST_BOTTOM_TEMP;
    drawScreen(ROOM_TEMP, EXHAUST_TOP_TEMP, EXHAUST_BOTTOM_TEMP, HIGH_TEMP_SET, LOW_TEMP_SET, 1, SELECTED_TEMP);
  }

  if (RESTART_TIME < millis() && !Task1_Running) {
    Serial.println("Restarting Temp Read Loop");
    vTaskResume(Task1);
    Task1_Running = true;
  }
  
  // startHeater();

  // Serial.println("On");
  // delay(1000);
  // stopHeater();
  // Serial.println("Off");
  // delay(1000);
}