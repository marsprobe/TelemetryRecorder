# TelemetryRecorder


Read telemetry data from MPU9250 (acceleration, rotation, compass), store it on SD card (in CSV format), and send/stream it via WiFi when in range.

The basic ideea is to place the platform on a motorbike/car and have it record all the movements, when getting home, in range of the WiFi, the thing should:
- send all the collected data to a base station.
- sync its clock with an NTP server and set the time on the RTC module
- use the RTC module for time reference when outside of WiFI range

#Components used:
- ESP8266 Wemos Mini D1
- MPU9250
- DS3231 RTC Module
- Voltage regulator (not yet determined)

#TODO
- File send mechanism.
- Control debug serial OUTPUT
- Add RTC support
- Add voltage measurement support
- Watchdog implementation ?
