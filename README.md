# Garage Heater Thermostat
Built for an ESP32

## Hardware
- 2 DS18B20 temperature sensors (room sensors - on board and below toolbox)
- 1 MAX6675 thermocouple (exhaust temperature monitoring)
- 3 relays: gas valve, exhaust fan, and blower fan
- OLED display (128x64 SSD1306)
- Rotary encoder for physical controls

## Features
- **WiFi Web Interface** - Control from phone/computer
- **OTA Updates** - Flash firmware over WiFi
- **Adjustable Cycle Times** - Tune gas on/off cycles via web UI
- **Temperature-Based Safety** - Exhaust temp monitoring with automatic cooldown
- **Sensor Validation** - Detects failed/stuck sensors and shuts down heater
- **Watchdog Timer** - Auto-recovery from hangs

## Safety Limits
- **Exhaust Temperature**: 140°F max (protects VIVOSUN 6" inline duct fan)
- **Room Sensors**: 145°F max (protects DS18B20 sensors rated to 150°F)
- **Sensor Range**: -20°F to 150°F (DS18B20), -40°F to 300°F (thermocouple)

## WiFi Setup
If no WiFi credentials are stored, the ESP32 boots in AP mode:
- **SSID**: `garage_heater`
- **Password**: None
- **IP Address**: `192.168.4.1`

Access the web interface to configure WiFi, then system will reboot and connect to your network.

## OTA Updates
**Network OTA (PlatformIO):**
```ini
; Add to platformio.ini
upload_protocol = espota
upload_port = Garage-Heater.local
upload_flags = --auth=garage123
```

**Web UI Upload:**
1. Navigate to Settings page
2. Upload `.bin` file
3. System reboots automatically

**Build firmware:**
```bash
pio run --target buildprog
# Binary: .pio/build/nodemcu-32s/firmware.bin
```

## Web Interface
- **Main Page**: Temperature display, mode controls, setpoint adjustments
- **Settings Page**: Cycle timing, firmware upload, WiFi configuration

## Default Settings
- **Gas On Time**: 5 minutes
- **Gas Rest Time**: 1.5 minutes
- **High Temp (Occupied)**: 68°F
- **Low Temp (Unoccupied)**: 38°F
