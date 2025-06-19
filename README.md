# ADXL345 Tap Detector with mDNS MQTT Discovery

A sophisticated ESP32-based tap detection system using the ADXL345 accelerometer with automatic MQTT broker discovery, adaptive thresholding, and comprehensive REST API control.

This scritp is perfect to help improve reliability on a Dartboard to work with [DartNet](https://github.com/syllebra/DartNet).

# TODO: Photos

## Features

### 🎯 Advanced Tap Detection

- **Adaptive Thresholding**: Automatically adjusts sensitivity based on environmental noise
- **Multi-criteria Detection**: Uses both magnitude and derivative (jerk) analysis
- **Configurable Parameters**: Fine-tune sensitivity, duration, and filtering
- **Low-pass Filtering**: Reduces noise for more accurate detection
- **State Machine**: Robust tap detection with cooldown periods

### 🌐 Network & Communication

- **mDNS Discovery**: Automatically finds MQTT brokers on the network (`_mqtt._tcp.local`)
- **MQTT Integration**: Publishes tap events with detailed metadata
- **REST API**: Complete configuration and monitoring interface
- **WiFi Connectivity**: Easy network setup

### 📊 Real-time Monitoring

- **Live Sensor Data**: Access accelerometer readings via API
- **Noise Level Tracking**: Monitor environmental conditions
- **Tap Statistics**: Duration, magnitude, and timing information
- **Debug Mode**: Detailed logging for troubleshooting

## Hardware Requirements

- **ESP32** development board
- **ADXL345** accelerometer module
- **I2C Connection**:
  - SDA → GPIO 21 (default)
  - SCL → GPIO 22 (default)
  - VCC → 3.3V
  - GND → GND

## Software Dependencies

### Arduino Libraries

```cpp
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <time.h>
#include <ESPmDNS.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
```

Install via Arduino Library Manager:

- `Adafruit ADXL345`
- `Adafruit MQTT Library`
- `ArduinoJson`

## Configuration

### 1. WiFi Setup

```cpp
const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";
```

### 2. MQTT Credentials

```cpp
const char *mqtt_user = "YOUR_MQTT_USER";
const char *mqtt_password = "YOUR_MQTT_PASSWORD";
const char *mqtt_topic_tap = "sensors/tap";
```

### 3. Timezone (Optional)

```cpp
const long gmtOffset_sec = 0;           // GMT offset in seconds
const int daylightOffset_sec = 3600;    // Daylight saving offset
```

## Installation

1. **Clone the repository**

   ```bash
   git clone https://github.com/syllebra/darts-impact-detector
   cd darts-impact-detector
   ```

2. **Open in Arduino IDE**

   - Install required libraries
   - Select ESP32 board
   - Configure WiFi and MQTT credentials

3. **Upload to ESP32**

   - Connect ESP32 via USB
   - Upload the sketch

4. **Wire the ADXL345**
   - Connect I2C pins as specified above
   - Power the sensor with 3.3V

## Usage

### Automatic MQTT Discovery

The system automatically discovers MQTT brokers on your network using mDNS. No need to hardcode IP addresses!

### REST API Endpoints

#### Configuration Management

- **GET** `/api/config` - Get current configuration
- **POST** `/api/config` - Update configuration parameters

#### Sensor Data

- **GET** `/api/sensor` - Get real-time accelerometer data

#### System Control

- **POST** `/api/reset` - Reset tap detection state machine
- **POST** `/api/calibrate` - Recalibrate baseline acceleration
- **POST** `/api/discover-mqtt` - Manually discover MQTT broker
- **GET** `/api/mqtt-info` - Get MQTT broker information

### Configuration Parameters

```json
{
  "threshold": 0.03,
  "min_delay_between_taps": 150,
  "tap_duration_min": 2,
  "tap_duration_max": 150,
  "enable_filtering": true,
  "filter_alpha": 0.7,
  "sensitivity": 50,
  "debug": false,
  "use_adaptive_threshold": true,
  "noise_window": 0.2,
  "peak_ratio": 1.2,
  "use_derivative": true,
  "derivative_threshold": 0.8,
  "enable_mdns_discovery": true
}
```

### Example API Usage

**Get current sensor data:**

```bash
curl http://tap-detector.local/api/sensor
```

**Update sensitivity:**

```bash
curl -X POST http://tap-detector.local/api/config \
  -H "Content-Type: application/json" \
  -d '{"sensitivity": 75}'
```

**Enable debug mode:**

```bash
curl -X POST http://tap-detector.local/api/config \
  -H "Content-Type: application/json" \
  -d '{"debug": true}'
```

## MQTT Message Format

When a tap is detected, the following JSON message is published:

```json
{
  "event": "tap",
  "timestamp": "2024-01-15T14:30:25",
  "magnitude": 1.234,
  "derivative": 0.567,
  "x": 0.123,
  "y": 0.456,
  "z": 0.789,
  "noise_level": 0.023,
  "threshold_used": 0.045,
  "device_time": 1234567890,
  "duration": 45
}
```

## Calibration

The system automatically calibrates on startup, but you can recalibrate:

1. **Place the sensor in its final position**
2. **Ensure it's stable and undisturbed**
3. **Call the calibration endpoint:**
   ```bash
   curl -X POST http://tap-detector.local/api/calibrate
   ```

## Troubleshooting

### Common Issues

**No tap detection:**

- Check calibration - sensor should be stable during startup
- Adjust sensitivity via API
- Enable debug mode to monitor thresholds
- Verify ADXL345 wiring and I2C communication

**False positives:**

- Increase `threshold` value
- Enable adaptive thresholding
- Reduce `sensitivity`
- Check for vibrations in the environment

**MQTT connection issues:**

- Verify broker is running and accessible
- Check credentials
- Use `/api/discover-mqtt` to find broker automatically
- Check firewall settings

**WiFi connectivity:**

- Verify SSID and password
- Check signal strength
- Monitor serial output for connection status

### Debug Mode

Enable debug mode for detailed monitoring:

```bash
curl -X POST http://tap-detector.local/api/config \
  -H "Content-Type: application/json" \
  -d '{"debug": true}'
```

This will output real-time sensor data every 0.5 seconds to the serial console.

## Advanced Configuration

### Adaptive Thresholding

The system can automatically adjust sensitivity based on environmental noise:

- `use_adaptive_threshold`: Enable/disable adaptive mode
- `peak_ratio`: Multiplier for noise-based threshold
- `noise_window`: Time window for noise calculation

### Filtering Options

- `enable_filtering`: Toggle low-pass filtering
- `filter_alpha`: Filter responsiveness (0.1-1.0)

### Detection Criteria

- `use_derivative`: Enable jerk-based detection
- `derivative_threshold`: Sensitivity for rate of change

## Performance

- **Sample Rate**: 100 Hz
- **Response Time**: < 10ms
- **Power Consumption**: ~50mA (ESP32 + ADXL345)
- **Memory Usage**: ~200KB program, ~50KB RAM

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## Support

- **Issues**: [GitHub Issues](https://github.com/syllebra/darts-impact-detector/issues)
- **Discussions**: [GitHub Discussions](https://github.com/syllebra/darts-impact-detector/discussions)

## Changelog

### v1.0.0

- Initial release with basic tap detection
- REST API configuration
- MQTT integration
- mDNS broker discovery
- Adaptive thresholding
- Comprehensive filtering options
