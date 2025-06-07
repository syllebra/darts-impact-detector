#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <time.h>
#include "SparkFun_ADXL345.h"

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT settings
const char* mqtt_server = "YOUR_MQTT_BROKER";
const int mqtt_port = 1883;
const char* mqtt_user = "YOUR_MQTT_USER";
const char* mqtt_password = "YOUR_MQTT_PASSWORD";
const char* mqtt_topic_tap = "sensors/tap";
const char* mqtt_topic_double_tap = "sensors/double_tap";

// NTP settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

// Objects
ADXL345 adxl = ADXL345();
WiFiClient espClient;
PubSubClient client(espClient);
WebServer server(80);

// Tap detection parameters (configurable via REST API)
struct TapConfig {
  float threshold = 2.5;           // Tap threshold in g
  float double_tap_window = 300;   // Double tap time window in ms
  float tap_duration_min = 10;     // Minimum tap duration in ms
  float tap_duration_max = 100;    // Maximum tap duration in ms
  bool enable_filtering = true;    // Enable low-pass filtering
  float filter_alpha = 0.3;       // Low-pass filter coefficient
  int sensitivity = 50;            // Overall sensitivity (1-100)
} tapConfig;

// Accelerometer data and filtering
struct AccelData {
  float x, y, z;
  float magnitude;
  unsigned long timestamp;
};

AccelData currentAccel, filteredAccel, prevAccel;
float baseline_magnitude = 1.0; // Earth's gravity baseline

// Tap detection state
enum TapState {
  IDLE,
  TAP_DETECTED,
  WAITING_FOR_DOUBLE
};

TapState tapState = IDLE;
unsigned long lastTapTime = 0;
unsigned long tapStartTime = 0;
bool tapInProgress = false;

// Sampling and timing
const int SAMPLE_RATE = 100; // Hz
const int SAMPLE_INTERVAL = 1000 / SAMPLE_RATE; // ms
unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C and ADXL345
  Wire.begin();
  adxl.powerOn();
  adxl.setRangeSetting(4); // 4g range
  adxl.setSpiBit(0);
  adxl.setActivityXYZ(1, 1, 1);
  adxl.setActivityThreshold(75);
  adxl.setInactivityXYZ(1, 1, 1);
  adxl.setInactivityThreshold(75);
  adxl.setTimeInactivity(10);
  
  // Connect to WiFi
  setupWiFi();
  
  // Initialize NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  
  // Setup REST API endpoints
  setupRestAPI();
  
  // Initialize baseline
  calibrateBaseline();
  
  Serial.println("ADXL345 Tap Detector Ready");
}

void loop() {
  // Maintain connections
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  server.handleClient();
  
  // Sample accelerometer at fixed rate
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL) {
    sampleAccelerometer();
    processTapDetection();
    lastSampleTime = millis();
  }
  
  delay(1);
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("WiFi connected! IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    String clientId = "TapDetector-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setupRestAPI() {
  // Get current configuration
  server.on("/api/config", HTTP_GET, []() {
    DynamicJsonDocument doc(1024);
    doc["threshold"] = tapConfig.threshold;
    doc["double_tap_window"] = tapConfig.double_tap_window;
    doc["tap_duration_min"] = tapConfig.tap_duration_min;
    doc["tap_duration_max"] = tapConfig.tap_duration_max;
    doc["enable_filtering"] = tapConfig.enable_filtering;
    doc["filter_alpha"] = tapConfig.filter_alpha;
    doc["sensitivity"] = tapConfig.sensitivity;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // Update configuration
  server.on("/api/config", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, server.arg("plain"));
      
      if (doc.containsKey("threshold")) tapConfig.threshold = doc["threshold"];
      if (doc.containsKey("double_tap_window")) tapConfig.double_tap_window = doc["double_tap_window"];
      if (doc.containsKey("tap_duration_min")) tapConfig.tap_duration_min = doc["tap_duration_min"];
      if (doc.containsKey("tap_duration_max")) tapConfig.tap_duration_max = doc["tap_duration_max"];
      if (doc.containsKey("enable_filtering")) tapConfig.enable_filtering = doc["enable_filtering"];
      if (doc.containsKey("filter_alpha")) tapConfig.filter_alpha = doc["filter_alpha"];
      if (doc.containsKey("sensitivity")) tapConfig.sensitivity = doc["sensitivity"];
      
      server.send(200, "application/json", "{\"status\":\"updated\"}");
      Serial.println("Configuration updated via REST API");
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    }
  });
  
  // Get current sensor readings
  server.on("/api/sensor", HTTP_GET, []() {
    DynamicJsonDocument doc(512);
    doc["x"] = currentAccel.x;
    doc["y"] = currentAccel.y;
    doc["z"] = currentAccel.z;
    doc["magnitude"] = currentAccel.magnitude;
    doc["timestamp"] = getTimestamp();
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // Calibrate baseline
  server.on("/api/calibrate", HTTP_POST, []() {
    calibrateBaseline();
    server.send(200, "application/json", "{\"status\":\"calibrated\"}");
  });
  
  server.begin();
  Serial.println("REST API server started on port 80");
}

void sampleAccelerometer() {
  int x, y, z;
  adxl.readAccel(&x, &y, &z);
  
  // Convert to g-force
  prevAccel = currentAccel;
  currentAccel.x = x * 0.00390625; // 4mg/LSB for ±4g range
  currentAccel.y = y * 0.00390625;
  currentAccel.z = z * 0.00390625;
  currentAccel.magnitude = sqrt(currentAccel.x * currentAccel.x + 
                               currentAccel.y * currentAccel.y + 
                               currentAccel.z * currentAccel.z);
  currentAccel.timestamp = millis();
  
  // Apply low-pass filter if enabled
  if (tapConfig.enable_filtering) {
    filteredAccel.x = applyLowPassFilter(currentAccel.x, filteredAccel.x);
    filteredAccel.y = applyLowPassFilter(currentAccel.y, filteredAccel.y);
    filteredAccel.z = applyLowPassFilter(currentAccel.z, filteredAccel.z);
    filteredAccel.magnitude = sqrt(filteredAccel.x * filteredAccel.x + 
                                  filteredAccel.y * filteredAccel.y + 
                                  filteredAccel.z * filteredAccel.z);
  } else {
    filteredAccel = currentAccel;
  }
}

float applyLowPassFilter(float current, float previous) {
  return tapConfig.filter_alpha * current + (1.0 - tapConfig.filter_alpha) * previous;
}

void processTapDetection() {
  float magnitude = filteredAccel.magnitude;
  float deviation = abs(magnitude - baseline_magnitude);
  float adjustedThreshold = tapConfig.threshold * (tapConfig.sensitivity / 100.0);
  
  unsigned long currentTime = millis();
  
  switch (tapState) {
    case IDLE:
      if (deviation > adjustedThreshold && !tapInProgress) {
        tapInProgress = true;
        tapStartTime = currentTime;
        tapState = TAP_DETECTED;
      }
      break;
      
    case TAP_DETECTED:
      if (tapInProgress) {
        unsigned long tapDuration = currentTime - tapStartTime;
        
        // Check if tap ended (acceleration returned to baseline)
        if (deviation < adjustedThreshold * 0.5) {
          if (tapDuration >= tapConfig.tap_duration_min && 
              tapDuration <= tapConfig.tap_duration_max) {
            // Valid tap detected
            handleTapEvent(tapStartTime);
            lastTapTime = tapStartTime;
            tapState = WAITING_FOR_DOUBLE;
          }
          tapInProgress = false;
        }
        // Timeout for too long tap
        else if (tapDuration > tapConfig.tap_duration_max) {
          tapInProgress = false;
          tapState = IDLE;
        }
      }
      break;
      
    case WAITING_FOR_DOUBLE:
      if (deviation > adjustedThreshold && !tapInProgress) {
        unsigned long timeSinceLastTap = currentTime - lastTapTime;
        
        if (timeSinceLastTap <= tapConfig.double_tap_window) {
          tapInProgress = true;
          tapStartTime = currentTime;
        } else {
          // Timeout, treat as new single tap
          tapInProgress = true;
          tapStartTime = currentTime;
          tapState = TAP_DETECTED;
        }
      }
      // Check for double tap completion
      else if (tapInProgress && deviation < adjustedThreshold * 0.5) {
        unsigned long tapDuration = currentTime - tapStartTime;
        unsigned long timeSinceLastTap = tapStartTime - lastTapTime;
        
        if (tapDuration >= tapConfig.tap_duration_min && 
            tapDuration <= tapConfig.tap_duration_max &&
            timeSinceLastTap <= tapConfig.double_tap_window) {
          // Valid double tap detected
          handleDoubleTapEvent(lastTapTime, tapStartTime);
        }
        tapInProgress = false;
        tapState = IDLE;
      }
      // Timeout for double tap window
      else if (currentTime - lastTapTime > tapConfig.double_tap_window && !tapInProgress) {
        tapState = IDLE;
      }
      break;
  }
}

void handleTapEvent(unsigned long timestamp) {
  String timestampStr = getTimestamp();
  
  DynamicJsonDocument doc(512);
  doc["event"] = "tap";
  doc["timestamp"] = timestampStr;
  doc["magnitude"] = filteredAccel.magnitude;
  doc["x"] = filteredAccel.x;
  doc["y"] = filteredAccel.y;
  doc["z"] = filteredAccel.z;
  doc["device_time"] = timestamp;
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(mqtt_topic_tap, payload.c_str());
  
  Serial.println("TAP detected at " + timestampStr);
  Serial.println("Payload: " + payload);
}

void handleDoubleTapEvent(unsigned long firstTap, unsigned long secondTap) {
  String timestampStr = getTimestamp();
  
  DynamicJsonDocument doc(512);
  doc["event"] = "double_tap";
  doc["timestamp"] = timestampStr;
  doc["first_tap_time"] = firstTap;
  doc["second_tap_time"] = secondTap;
  doc["interval"] = secondTap - firstTap;
  doc["magnitude"] = filteredAccel.magnitude;
  doc["x"] = filteredAccel.x;
  doc["y"] = filteredAccel.y;
  doc["z"] = filteredAccel.z;
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(mqtt_topic_double_tap, payload.c_str());
  
  Serial.println("DOUBLE TAP detected at " + timestampStr);
  Serial.println("Payload: " + payload);
}

void calibrateBaseline() {
  Serial.println("Calibrating baseline...");
  float sum = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    int x, y, z;
    adxl.readAccel(&x, &y, &z);
    float magnitude = sqrt(pow(x * 0.00390625, 2) + 
                          pow(y * 0.00390625, 2) + 
                          pow(z * 0.00390625, 2));
    sum += magnitude;
    delay(10);
  }
  
  baseline_magnitude = sum / samples;
  Serial.println("Baseline magnitude: " + String(baseline_magnitude) + " g");
}

String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return String(millis()); // Fallback to millis if NTP fails
  }
  
  char buffer[64];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  return String(buffer);
}
