# Garage Heater Thermostat
Built for an ESP32

Uses 2 DS18B20 temperature sensors and one MAX6675 thermocouple.

My garage heater is a little unique where I have a powered exhaust fan and I wanted to ensure the temperature didn't exceed the limitations of the fan (260°F if I remember correctly), hence the additional temperature sensors.

Three relays: gas, exhaust fan, and blower fan

Uses parameters to store wifi credentials. If there are no parameters stored (aka a fresh system), the ESP32 boots wifi in AP mode and creates an SSID of "garage-heater" with no passweord. Default IP address for AP mode is the Arduino default of 192.168.4.1.
