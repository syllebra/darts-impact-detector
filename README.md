# darts-impact-detector
esp32 code to detect darts impacts on a regular dartboard using ADXL345 accelerometer

Here's what the script includes:

##Key Features:
###1. Tap Detection Algorithm:

3-axis accelerometer filtering with configurable low-pass filter
Magnitude-based tap detection with baseline calibration
Separate handling for single taps and double taps
Configurable tap duration windows and thresholds

###2. MQTT Communication:

Publishes tap events to sensors/tap topic
Publishes double-tap events to sensors/double_tap topic
JSON payload includes timestamp, acceleration data, and event details

###3. REST API Configuration:

GET /api/config - Get current detector parameters
POST /api/config - Update detector parameters
GET /api/sensor - Get current sensor readings
POST /api/calibrate - Recalibrate baseline

###4. Timestamping:

NTP synchronization for accurate timestamps
ISO 8601 format timestamps
Fallback to device uptime if NTP unavailable

##Configuration Parameters:

threshold: Tap detection sensitivity (g-force)
double_tap_window: Maximum time between taps for double-tap (ms)
tap_duration_min/max: Valid tap duration range (ms)
filter_alpha: Low-pass filter coefficient (0-1)
sensitivity: Overall sensitivity scaling (1-100)

##Setup Requirements:

Libraries needed:
cpp// Install these libraries in Arduino IDE:
- WiFi (built-in)
- WebServer (built-in)  
- PubSubClient
- ArduinoJson
- SparkFun ADXL345

##Hardware connections:

ADXL345 VCC → 3.3V
ADXL345 GND → GND
ADXL345 SDA → GPIO 21 (ESP32)
ADXL345 SCL → GPIO 22 (ESP32)


##Configuration:

Update WiFi credentials
Set MQTT broker details
Adjust NTP server if needed



##Usage Examples:
REST API Configuration:
```
# Get current config
curl http://[ESP32_IP]/api/config
```

# Update sensitivity
```
curl -X POST http://[ESP32_IP]/api/config \
  -H "Content-Type: application/json" \
  -d '{"sensitivity": 75, "threshold": 2.0}'
```
MQTT Message Format:
```
json{
  "event": "tap",
  "timestamp": "2025-06-06T10:30:45",
  "magnitude": 2.8,
  "x": 0.5, "y": 1.2, "z": 2.1,
  "device_time": 12345
}
```
The script includes proper state machine logic for reliable tap detection, configurable filtering to reduce noise, and comprehensive error handling for network connectivity.
