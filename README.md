# TelemetryRecorder


Read telemetry data from MPU9250 (acceleration, rotation, compass), store it on SD card (in CSV format), and send/stream it via WiFi when in range.

The basic ideea is to place the platform on a motorbike/car and have it record all the movements on a SD card. When arriving home, in range of the WiFi, the thing should:
- send all the collected data to a FTP server.
- sync its clock with an NTP server and set the time on the RTC module
- use the RTC module for time reference when outside of WiFi range

# Components used:

- ESP8266 Wemos Mini D1
- MPU9250
- DS3231 RTC Module
- Micro SD Shield for WeMos D1 mini TF
- Voltage regulator (not yet determined)

# TODO

- Control debug serial OUTPUT
- Add voltage measurement support
- Watchdog implementation ?
