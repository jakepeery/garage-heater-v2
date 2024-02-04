#include <Preferences.h>

#include "ArduinoJson.h"
#include "AsyncJson.h"
#include "ESPAsyncWebServer.h"
#include "WiFi.h"
#include "main.h"

#define ServerPort 80

String ssid;
String password;

bool AP_Mode;

static Preferences storedWifi;

AsyncWebServer server(ServerPort);

String get_encryption_type(wifi_auth_mode_t encryptionType) {
  switch (encryptionType) {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
    case (WIFI_AUTH_MAX):
      return "WIFI_MAX";
  }
  return "Unknown";
}

String get_wifi_status(int status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return "WL_IDLE_STATUS";
    case WL_SCAN_COMPLETED:
      return "WL_SCAN_COMPLETED";
    case WL_NO_SSID_AVAIL:
      return "WL_NO_SSID_AVAIL";
    case WL_CONNECT_FAILED:
      return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "WL_CONNECTION_LOST";
    case WL_CONNECTED:
      return "WL_CONNECTED";
    case WL_DISCONNECTED:
      return "WL_DISCONNECTED";
  }
  return "Unknown";
}

void SaveWifiInfo(String toSaveSSID, String toSavePassword) {
  Serial.println("\n\nSaving wifi info to flash");
  Serial.print("SSID: ");
  Serial.print(toSaveSSID);
  Serial.print("  Password: ");
  Serial.println(toSavePassword);

  storedWifi.begin("Wifi", false);
  storedWifi.putString("ssid", toSaveSSID);
  storedWifi.putString("password", toSavePassword);
  storedWifi.end();
}

void ScanWifi() {
  Serial.println("uPesy WiFi Scan Demo");
  Serial.println("[*] Scanning WiFi network");

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks(true, true, false);
  Serial.println("[*] Scan done");
  if (n == 0) {
    Serial.println("[-] No WiFi networks found");
  } else {
    Serial.println((String) "[+] " + n + " WiFi networks found\n");
    for (int i = 0; i < n; ++i) {
      // Print SSID, RSSI and WiFi Encryption for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dB) [");
      Serial.print(get_encryption_type(WiFi.encryptionType(i)));
      Serial.println("]");
      delay(10);
    }
  }
  Serial.println("");

  // Wait a bit before scanning again
  delay(5000);
}

bool SetupWifi() {
  // Connect to Wi-Fi
  AP_Mode = false;

  // Read in parameters
  storedWifi.begin("Wifi", false);
  ssid = storedWifi.getString("ssid", "none");
  password = storedWifi.getString("password", "none");
  storedWifi.end();

  if (ssid == "none" && password == "none") {
    Serial.println("No SSID and password stored in Flash Memory");
    return false;
  }
  Serial.print("Using SSID: :");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);

  WiFi.disconnect();
  delay(500);

  WiFi.setHostname("Garage-Heater");
  WiFi.begin(ssid.c_str(), password.c_str());

  Serial.println("Attempting to connect to WiFi.. ");
  Serial.print("SSID: ");
  Serial.print(ssid);
  Serial.print(" password: ");
  Serial.println(password);

  int status = WL_IDLE_STATUS;
  Serial.println("\nConnecting");
  Serial.println(get_wifi_status(status));

  int count = 1;
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WiFi.. ");
    Serial.println(count);
    count = count + 1;

    delay(1000);
    status = WiFi.status();
    Serial.println(get_wifi_status(status));
    if (count > 10) {
      break;
    }
  }
  WiFi.hostname("Garage-Heater");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to Wifi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("Connected to Wifi FAILED!!!!!");
    return false;
  }
}

void SetupWifiAsAP() {
  const char *ssid = "garage_heater ";
  IPAddress Server_ip(192, 168, 4, 1);  // IP address of this box
  IPAddress gateway(192, 168, 4, 1);    // gateway of your network
  IPAddress subnet(255, 255, 255, 0);   // subnet mask of your network
  IPAddress dns(8, 8, 8, 8);            // dns address needed to help get to
                                        // internet

  WiFi.disconnect();
  delay(1000);
  AP_Mode = true;

  Serial.println("Starting WiFi in AP Mode.. ");
  Serial.print("SSID: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, NULL);
  delay(1000);

  WiFi.config(Server_ip, gateway, subnet);
  Serial.println("WiFi in Local AP Mode\n");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Replaces placeholder with LED state value
String processor(const String &var) {
  String ledState;
  Serial.println(var);
  if (var == "STATE") {
    if (true) {
      ledState = "ON";
    } else {
      ledState = "OFF";
    }
    Serial.print(ledState);
    return ledState;
  }
  return String();
}

// String GetTemps() {
//   AsyncJsonResponse *response = new AsyncJsonResponse();
//   response->addHeader("Server", "ESP Async Web Server");
//   JsonObject &root = response->getRoot();
//   root["heap"] = ESP.getFreeHeap();
//   root["ssid"] = WiFi.SSID();
//   response->setLength();
//   request->send(response);
// }

void SetupRoutes(UserSettableData *User) {
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to load style.css file
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/favicon.ico", "image/x-icon");
  });

  //   // Route to load style.css file
  //   server.on("/gear.svg", HTTP_GET, [](AsyncWebServerRequest *request) {
  //     request->send(SPIFFS, "/gear.svg", "image/x-icon");
  //   });

  // Route to set GPIO to HIGH
  server.on("/on", HTTP_GET, [User](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/cgi-bin/setWifi", HTTP_GET,
            [User](AsyncWebServerRequest *request) {
              String ssid;
              String password;
              AsyncJsonResponse *response = new AsyncJsonResponse();

              response->setContentType("application/json");
              response->addHeader("Server", "ESP Async Web Server");
              response->setCode(200);
              JsonObject root = response->getRoot();

              if (request->hasParam("ssid") && request->hasParam("password")) {
                ssid = request->getParam("ssid")->value();
                password = request->getParam("password")->value();
                Serial.println("\n\nStoring new Wifi Info");
                Serial.print("ssid: ");
                Serial.print(ssid);
                Serial.print("   password: ");
                Serial.println(password);

                SaveWifiInfo(ssid, password);
                request->redirect("/");
                vTaskDelay(5000);
                Serial.println("\nRebooting!!!!!\n\n");
                ESP.restart();  // restart to use the new wwifi paramaters
              } else {
                root["failed"] = "must send in ?mode=int value between 1 and 3";
                response->setCode(400);
                response->setLength();
                request->send(response);
              }
            });

  server.on("/cgi-bin/setMode", HTTP_GET,
            [User](AsyncWebServerRequest *request) {
              String param;
              AsyncJsonResponse *response = new AsyncJsonResponse();

              response->setContentType("application/json");
              response->addHeader("Server", "ESP Async Web Server");
              response->setCode(200);
              JsonObject root = response->getRoot();

              int selected_mode = 0;
              if (request->hasParam("mode")) {
                param = request->getParam("mode")->value();
                selected_mode = param.toInt();
              } else {
                root["failed"] = "must send in ?mode=int value between 1 and 3";
                response->setCode(400);
                response->setLength();
                request->send(response);
              }

              if (selected_mode == 1) {
                root["room_state"] = "UnOccupied";
              } else if (selected_mode == 2) {
                root["room_state"] = "Heat On";
              } else if (selected_mode == 3) {
                root["room_state"] = "Full Off";
              } else if (selected_mode == 4) {
                root["room_state"] = "Exhaust";
              } else if (selected_mode == 5) {
                root["room_state"] = "Force Fan";
              } else {
                root["room_state"] = "Unknown";
              }

              User->SELECTED_MODE = selected_mode;

              response->setLength();
              request->send(response);
            });

  server.on(
      "/cgi-bin/bumpHighTemp", HTTP_GET,
      [User](AsyncWebServerRequest *request) {
        String direction;
        AsyncJsonResponse *response = new AsyncJsonResponse();

        Serial.println("bump_high_temp");

        response->setContentType("application/json");
        response->addHeader("Server", "ESP Async Web Server");
        response->setCode(200);
        JsonObject root = response->getRoot();

        if (request->hasParam("direction")) {
          direction = request->getParam("direction")->value();
        } else {
          root["failed"] =
              "no parameter, must send in ?direction=up or &direction=down";
          response->setCode(400);
          response->setLength();
          request->send(response);
        }

        if (direction == "up" && User->HIGH_TEMP_SET <= 80) {
          root["changed_temp"] = direction;
          User->HIGH_TEMP_SET = User->HIGH_TEMP_SET + 1;
        } else if (direction == "down" && User->HIGH_TEMP_SET >= 32) {
          root["changed_temp"] = direction;
          User->HIGH_TEMP_SET = User->HIGH_TEMP_SET - 1;
        } else {
          root["failed"] = "must send in ?direction=up or &direction=down";
          response->setCode(400);
        }

        char temp[8];
        float temp_float = User->HIGH_TEMP_SET;
        sprintf(temp, "%.0f", temp_float);
        root["high_temp"] = temp;

        response->setLength();
        request->send(response);
      });

  server.on(
      "/cgi-bin/bumpLowTemp", HTTP_GET, [User](AsyncWebServerRequest *request) {
        Serial.println("bump_low_temp");

        String direction;
        AsyncJsonResponse *response = new AsyncJsonResponse();
        response->setContentType("application/json");
        response->addHeader("Server", "ESP Async Web Server");
        response->setCode(200);
        JsonObject root = response->getRoot();

        if (request->hasParam("direction")) {
          direction = request->getParam("direction")->value();
          Serial.print("parameter");
          Serial.println(direction);
        } else {
          root["failed"] =
              "no parameter, must send in ?direction=up or &direction=down";
          response->setCode(400);
          response->setLength();
          request->send(response);
        }

        if (direction == "up" && User->LOW_TEMP_SET <= 80) {
          root["low_temp"] = direction;
          User->LOW_TEMP_SET = User->LOW_TEMP_SET + 1;
        } else if (direction == "down" && User->LOW_TEMP_SET >= 32) {
          root["changed_temp"] = direction;
          User->LOW_TEMP_SET = User->LOW_TEMP_SET - 1;
        } else {
          root["failed"] = "must send in ?direction=up or &direction=down";
          response->setCode(400);
        }
        char temp[8];
        float temp_float = User->LOW_TEMP_SET;
        sprintf(temp, "%.0f", temp_float);
        root["low_temp"] = temp;

        response->setLength();
        request->send(response);
      });

  server.on(
      "/cgi-bin/setHighTemp", HTTP_GET, [User](AsyncWebServerRequest *request) {
        String level;
        AsyncJsonResponse *response = new AsyncJsonResponse();

        response->setContentType("application/json");
        response->addHeader("Server", "ESP Async Web Server");
        response->setCode(200);
        JsonObject root = response->getRoot();

        float level_float = 0;
        if (request->hasParam("level")) {
          level = request->getParam("level")->value();
          level_float = level.toFloat();
        } else {
          root["failed"] = "must send in ?level=value value between 32 and 80";
          response->setCode(400);
          response->setLength();
          request->send(response);
        }

        if (level_float >= 32 && level_float <= 80) {
          root["high_temp"] = level;
          User->HIGH_TEMP_SET = level_float;
        } else {
          root["failed"] = "must send in ?level=value value between 32 and 80";
          response->setCode(400);
        }
        char temp[8];
        float temp_float = User->HIGH_TEMP_SET;
        sprintf(temp, "%.0f", temp_float);
        root["high_temp"] = temp;

        response->setLength();
        request->send(response);
      });

  server.on(
      "/cgi-bin/setLowTemp", HTTP_GET, [User](AsyncWebServerRequest *request) {
        String level;
        AsyncJsonResponse *response = new AsyncJsonResponse();

        response->setContentType("application/json");
        response->addHeader("Server", "ESP Async Web Server");
        response->setCode(200);
        JsonObject root = response->getRoot();

        float level_float = 0;
        if (request->hasParam("level")) {
          level = request->getParam("level")->value();
          level_float = level.toFloat();
        } else {
          root["failed"] = "must send in ?level=value value between 32 and 80";
          response->setCode(400);
          response->setLength();
          request->send(response);
        }

        if (level_float >= 32 && level_float <= 80) {
          root["low_temp"] = level;
          User->LOW_TEMP_SET = level_float;
        } else {
          root["failed"] = "must send in ?level=value value between 32 and 80";
          response->setCode(400);
        }
        char temp[8];
        float temp_float = User->LOW_TEMP_SET;
        sprintf(temp, "%.0f", temp_float);
        root["low_temp"] = temp;

        response->setLength();
        request->send(response);
      });

  server.on("/cgi-bin/getTemps", HTTP_GET,
            [User](AsyncWebServerRequest *request) {
              AsyncJsonResponse *response = new AsyncJsonResponse();

              response->setContentType("application/json");
              response->addHeader("Server", "ESP Async Web Server");
              response->setCode(200);
              JsonObject root = response->getRoot();

              char top_temp[8];
              char bottom_temp[8];
              char room_temp[8];

              char high_temp[8];
              char low_temp[8];

              float top = User->SENSOR_TEMPS.EXHAUST_TOP_TEMP;
              float bottom = User->SENSOR_TEMPS.EXHAUST_BOTTOM_TEMP;
              float room = User->SENSOR_TEMPS.ROOM_TEMP;

              float high = User->HIGH_TEMP_SET;
              float low = User->LOW_TEMP_SET;

              int selected_mode = User->SELECTED_MODE;

              sprintf(top_temp, "%.1f", top);
              sprintf(bottom_temp, "%.1f", bottom);
              sprintf(room_temp, "%.1f", room);

              sprintf(high_temp, "%.0f", high);
              sprintf(low_temp, "%.0f", low);

              root["exhaust_top_temp"] = top_temp;
              root["exhaust_bottom_temp"] = bottom_temp;
              root["room_temp"] = room_temp;
              root["gas_on"] = User->GAS_ON;
              root["fan_on"] = User->FAN_ON;
              root["exhaust_on"] = User->EXHAUST_ON;

              root["high_temp"] = high_temp;
              root["low_temp"] = low_temp;

              if (selected_mode == 1) {
                root["room_state"] = "UnOccupied";
              } else if (selected_mode == 2) {
                root["room_state"] = "Heat On";
              } else if (selected_mode == 3) {
                root["room_state"] = "Full Off";
              } else {
                root["room_state"] = "Unknown";
              }

              response->setLength();
              request->send(response);
            });
}

// String IpAddress2String(const IPAddress &ipAddress) {
//   return String(ipAddress[0]) + String(".") + String(ipAddress[1]) +
//          String(".") + String(ipAddress[2]) + String(".") +
//          String(ipAddress[3]);
// }

void SetupWebServerWithWifi(UserSettableData *UserData) {
  bool SetupWithWifi = SetupWifi();
  Serial.print("SetupWithWifi: ");
  Serial.println(SetupWithWifi);
  AP_Mode = false;
  if (SetupWithWifi == false) {
    Serial.println("Attempting to start WiFi in AP Mode.. ");
    AP_Mode = true;
    SetupWifiAsAP();
  }
  delay(1000);
  SetupRoutes(UserData);

  String ipAddressReturn;
  ipAddressReturn = WiFi.localIP().toString();
  strcpy(UserData->IPAddress, ipAddressReturn.c_str());

  server.begin();
}

// checks to make sure wifi is still connected and reconnects if failed
void MaintainWifi(void *pvParameters) {
  while (1) {
    if (WiFi.status() == WL_CONNECTED || AP_Mode) {
      Serial.println("\nConnected to Wifi Check");
      if (AP_Mode) {
        Serial.println("In Access Point Mode");
      }
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      vTaskDelay(10000);
    } else {
      Serial.println("Connected to Wifi FAILED!!!!!");
      Serial.print("WiFi.status() ");
      Serial.println(WiFi.status());
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.println("Reconnecting to Wifi...");
      SetupWifi();
      vTaskDelay(30000);  // wait longer if trying to reconnect
    }
  }
}