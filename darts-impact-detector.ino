#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <time.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// WiFi credentials
const char *ssid = "WIFI_SSID";
const char *password = "WIFI_PASSWORD";

// MQTT settings
const char *mqtt_server = "MQTT_SERVER";
const int mqtt_port = 1883;
const char *mqtt_user = "YOUR_MQTT_USER";
const char *mqtt_password = "YOUR_MQTT_PASSWORD";
const char *mqtt_topic_tap = "sensors/tap";

// NTP settings
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

// Objects
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, mqtt_server, mqtt_port, mqtt_user, mqtt_password);
Adafruit_MQTT_Publish tapFeed = Adafruit_MQTT_Publish(&mqtt, mqtt_topic_tap);
WebServer server(80);

// Tap detection parameters (configurable via REST API)
struct TapConfig
{
  float threshold = 0.03;             // Base tap threshold in g
  float min_delay_between_taps = 150; // Minimum delay between taps in ms
  float tap_duration_min = 2;         // Minimum tap duration in ms
  float tap_duration_max = 150;       // Maximum tap duration in ms
  bool enable_filtering = true;       // Enable low-pass filtering
  float filter_alpha = 0.7;           // Low-pass filter coefficient (higher = more responsive)
  int sensitivity = 50;               // Overall sensitivity (1-100)
  bool debug = false;                 // Debug output
  bool use_adaptive_threshold = true; // Use adaptive threshold based on noise level
  float noise_window = 0.2;           // Window for noise calculation in seconds
  float peak_ratio = 1.2;             // Ratio of peak to noise for detection
  bool use_derivative = true;         // Use derivative (jerk) for better detection
  float derivative_threshold = 0.8;   // Threshold for derivative detection
} tapConfig;

// Accelerometer data and filtering
struct AccelData
{
  float x, y, z;
  float magnitude;
  float derivative;
  unsigned long timestamp;
};

AccelData currentAccel, filteredAccel, prevAccel;
float baseline_magnitude = 1.0;
float noise_level = 0.02; // Running estimate of noise level

// Tap detection state
enum TapState
{
  IDLE,
  TAP_DETECTED,
  COOLDOWN
};

TapState tapState = IDLE;
unsigned long lastTapTime = 0;
unsigned long tapStartTime = 0;
unsigned long stateChangeTime = 0;
bool tapInProgress = false;

// Noise and adaptive threshold tracking
const int NOISE_BUFFER_SIZE = 200; // 2 seconds at 100Hz
float noiseBuffer[NOISE_BUFFER_SIZE];
int noiseBufferIndex = 0;
bool noiseBufferFull = false;

// Sampling and timing
const int SAMPLE_RATE = 100;                    // Hz
const int SAMPLE_INTERVAL = 1000 / SAMPLE_RATE; // ms
unsigned long lastSampleTime = 0;

void setup()
{
  Serial.begin(115200);

  // Initialize I2C and ADXL345
  Wire.begin();
  if (!accel.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while (1)
      ;
  }

  // Set range to 4G for better sensitivity
  accel.setRange(ADXL345_RANGE_4_G);

  // Connect to WiFi
  setupWiFi();

  // Initialize NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Setup REST API endpoints
  setupRestAPI();

  // Initialize baseline and noise buffer
  calibrateBaseline();
  initializeNoiseBuffer();

  // Initialize state tracking
  stateChangeTime = millis();

  Serial.println("ADXL345 Tap Detector Ready (Simplified)");
}

int debugCounter = 0;
void loop()
{
  // Maintain MQTT connection
  MQTT_connect();

  server.handleClient();

  // Sample accelerometer at fixed rate
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL)
  {
    sampleAccelerometer();
    updateNoiseLevel();
    processTapDetection();
    lastSampleTime = millis();
  }

  // Debug output
  if (tapConfig.debug)
  {
    debugCounter++;
    if (debugCounter == 50)
    { // Every 0.5 seconds
      Serial.printf("Mag: %.3f | Deriv: %.3f | Noise: %.3f | Thresh: %.3f | State: %d\n",
                    filteredAccel.magnitude, filteredAccel.derivative,
                    noise_level, getAdaptiveThreshold(), tapState);
      debugCounter = 0;
    }
  }

  delay(1);
}

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi connected! IP address: ");
  Serial.println(WiFi.localIP());
}

void MQTT_connect()
{
  int8_t ret;

  if (mqtt.connected())
  {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0)
  {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0)
    {
      Serial.println("MQTT connection failed, continuing without MQTT");
      return;
    }
  }
  Serial.println("MQTT Connected!");
}

void setupRestAPI()
{
  // Get current configuration
  server.on("/api/config", HTTP_GET, []()
            {
    DynamicJsonDocument doc(1024);
    doc["threshold"] = tapConfig.threshold;
    doc["min_delay_between_taps"] = tapConfig.min_delay_between_taps;
    doc["tap_duration_min"] = tapConfig.tap_duration_min;
    doc["tap_duration_max"] = tapConfig.tap_duration_max;
    doc["enable_filtering"] = tapConfig.enable_filtering;
    doc["filter_alpha"] = tapConfig.filter_alpha;
    doc["sensitivity"] = tapConfig.sensitivity;
    doc["debug"] = tapConfig.debug;
    doc["use_adaptive_threshold"] = tapConfig.use_adaptive_threshold;
    doc["noise_window"] = tapConfig.noise_window;
    doc["peak_ratio"] = tapConfig.peak_ratio;
    doc["use_derivative"] = tapConfig.use_derivative;
    doc["derivative_threshold"] = tapConfig.derivative_threshold;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response); });

  // Update configuration
  server.on("/api/config", HTTP_POST, []()
            {
    if (server.hasArg("plain")) {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, server.arg("plain"));
      
      if (doc.containsKey("threshold")) tapConfig.threshold = doc["threshold"];
      if (doc.containsKey("min_delay_between_taps")) tapConfig.min_delay_between_taps = doc["min_delay_between_taps"];
      if (doc.containsKey("tap_duration_min")) tapConfig.tap_duration_min = doc["tap_duration_min"];
      if (doc.containsKey("tap_duration_max")) tapConfig.tap_duration_max = doc["tap_duration_max"];
      if (doc.containsKey("enable_filtering")) tapConfig.enable_filtering = doc["enable_filtering"];
      if (doc.containsKey("filter_alpha")) tapConfig.filter_alpha = doc["filter_alpha"];
      if (doc.containsKey("sensitivity")) tapConfig.sensitivity = doc["sensitivity"];
      if (doc.containsKey("debug")) tapConfig.debug = doc["debug"];
      if (doc.containsKey("use_adaptive_threshold")) tapConfig.use_adaptive_threshold = doc["use_adaptive_threshold"];
      if (doc.containsKey("noise_window")) tapConfig.noise_window = doc["noise_window"];
      if (doc.containsKey("peak_ratio")) tapConfig.peak_ratio = doc["peak_ratio"];
      if (doc.containsKey("use_derivative")) tapConfig.use_derivative = doc["use_derivative"];
      if (doc.containsKey("derivative_threshold")) tapConfig.derivative_threshold = doc["derivative_threshold"];
      
      server.send(200, "application/json", "{\"status\":\"updated\"}");
      Serial.println("Configuration updated via REST API");
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    } });

  // Get current sensor readings
  server.on("/api/sensor", HTTP_GET, []()
            {
    DynamicJsonDocument doc(512);
    doc["x"] = currentAccel.x;
    doc["y"] = currentAccel.y;
    doc["z"] = currentAccel.z;
    doc["magnitude"] = currentAccel.magnitude;
    doc["derivative"] = currentAccel.derivative;
    doc["filtered_magnitude"] = filteredAccel.magnitude;
    doc["noise_level"] = noise_level;
    doc["adaptive_threshold"] = getAdaptiveThreshold();
    doc["timestamp"] = getTimestamp();
    doc["state"] = tapState;
    doc["time_since_last_tap"] = millis() - lastTapTime;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response); });

  // Reset state machine
  server.on("/api/reset", HTTP_POST, []()
            {
    resetStateMachine();
    server.send(200, "application/json", "{\"status\":\"reset\"}"); });

  // Calibrate baseline
  server.on("/api/calibrate", HTTP_POST, []()
            {
    calibrateBaseline();
    initializeNoiseBuffer();
    server.send(200, "application/json", "{\"status\":\"calibrated\"}"); });

  server.begin();
  Serial.println("REST API server started on port 80");
}

void sampleAccelerometer()
{
  sensors_event_t event;
  accel.getEvent(&event);

  // Store previous reading
  prevAccel = currentAccel;

  // Update current reading
  currentAccel.x = event.acceleration.x / 9.81; // Convert m/s² to g
  currentAccel.y = event.acceleration.y / 9.81;
  currentAccel.z = event.acceleration.z / 9.81;
  currentAccel.magnitude = sqrt(currentAccel.x * currentAccel.x +
                                currentAccel.y * currentAccel.y +
                                currentAccel.z * currentAccel.z);
  currentAccel.timestamp = millis();

  // Calculate derivative (jerk) for better tap detection
  if (prevAccel.timestamp > 0)
  {
    float dt = (currentAccel.timestamp - prevAccel.timestamp) / 1000.0; // Convert to seconds
    if (dt > 0)
    {
      currentAccel.derivative = abs(currentAccel.magnitude - prevAccel.magnitude) / dt;
    }
  }

  // Apply low-pass filter if enabled
  if (tapConfig.enable_filtering)
  {
    filteredAccel.x = applyLowPassFilter(currentAccel.x, filteredAccel.x);
    filteredAccel.y = applyLowPassFilter(currentAccel.y, filteredAccel.y);
    filteredAccel.z = applyLowPassFilter(currentAccel.z, filteredAccel.z);
    filteredAccel.magnitude = sqrt(filteredAccel.x * filteredAccel.x +
                                   filteredAccel.y * filteredAccel.y +
                                   filteredAccel.z * filteredAccel.z);
    filteredAccel.derivative = applyLowPassFilter(currentAccel.derivative, filteredAccel.derivative);
  }
  else
  {
    filteredAccel = currentAccel;
  }
}

float applyLowPassFilter(float current, float previous)
{
  return tapConfig.filter_alpha * current + (1.0 - tapConfig.filter_alpha) * previous;
}

void updateNoiseLevel()
{
  if (tapState != IDLE)
    return;
  // Add current deviation from baseline to noise buffer
  float deviation = abs(filteredAccel.magnitude - baseline_magnitude);
  noiseBuffer[noiseBufferIndex] = deviation;
  noiseBufferIndex = (noiseBufferIndex + 1) % NOISE_BUFFER_SIZE;

  if (noiseBufferIndex == 0)
  {
    noiseBufferFull = true;
  }

  // Calculate running noise level (RMS)
  float sum = 0;
  int samples = noiseBufferFull ? NOISE_BUFFER_SIZE : noiseBufferIndex;
  for (int i = 0; i < samples; i++)
  {
    sum += noiseBuffer[i] * noiseBuffer[i];
  }
  noise_level = sqrt(sum / samples);
}

float getAdaptiveThreshold()
{
  if (tapConfig.use_adaptive_threshold)
  {
    float base_threshold = tapConfig.threshold * (tapConfig.sensitivity / 100.0f);
    float noise_threshold = noise_level * tapConfig.peak_ratio;
    return max(base_threshold, noise_threshold);
  }
  else
  {
    return tapConfig.threshold * (tapConfig.sensitivity / 100.0f);
  }
}

bool detectTapSignature()
{
  float magnitude_deviation = abs(filteredAccel.magnitude - baseline_magnitude);
  float threshold = getAdaptiveThreshold();

  // Primary detection: magnitude threshold
  bool magnitude_trigger = magnitude_deviation > threshold;

  // Secondary detection: derivative (jerk) threshold
  bool derivative_trigger = false;
  if (tapConfig.use_derivative && filteredAccel.derivative > 0)
  {
    derivative_trigger = filteredAccel.derivative > tapConfig.derivative_threshold;
  }

  // Combine detections based on configuration
  if (tapConfig.use_derivative)
  {
    return magnitude_trigger && derivative_trigger;
  }
  else
  {
    return magnitude_trigger;
  }
}

void processTapDetection()
{
  unsigned long currentTime = millis();

  switch (tapState)
  {
  case IDLE:
    if (detectTapSignature() && !tapInProgress)
    {
      // Check minimum delay since last tap
      if (currentTime - lastTapTime >= tapConfig.min_delay_between_taps)
      {
        tapInProgress = true;
        tapStartTime = currentTime;
        changeState(TAP_DETECTED);
      }
    }
    break;

  case TAP_DETECTED:
    if (tapInProgress)
    {
      unsigned long tapDuration = currentTime - tapStartTime;

      // Check if tap ended (signals returned to baseline)
      if (!detectTapSignature())
      {
        tapInProgress = false;

        if (tapDuration >= tapConfig.tap_duration_min &&
            tapDuration <= tapConfig.tap_duration_max)
        {
          // Valid tap detected
          handleTapEvent();
          lastTapTime = currentTime;
          changeState(COOLDOWN);
        }
        else
        {
          // Invalid tap duration
          changeState(IDLE);
        }
      }
      // Timeout for too long tap
      else if (tapDuration > tapConfig.tap_duration_max)
      {
        tapInProgress = false;
        changeState(IDLE);
      }
    }
    else
    {
      // Shouldn't be here without tapInProgress
      changeState(IDLE);
    }
    break;

  case COOLDOWN:
    // Brief cooldown period to prevent immediate re-triggering
    if (currentTime - stateChangeTime >= 50)
    { // 50ms cooldown
      changeState(IDLE);
    }
    break;
  }
}

void changeState(TapState newState)
{
  if (tapState != newState)
  {
    tapState = newState;
    stateChangeTime = millis();
  }
}

void resetStateMachine()
{
  tapState = IDLE;
  tapInProgress = false;
  stateChangeTime = millis();
  Serial.println("State machine reset to IDLE");
}

void handleTapEvent()
{
  String timestampStr = getTimestamp();

  DynamicJsonDocument doc(512);
  doc["event"] = "tap";
  doc["timestamp"] = timestampStr;
  doc["magnitude"] = filteredAccel.magnitude;
  doc["derivative"] = filteredAccel.derivative;
  doc["x"] = filteredAccel.x;
  doc["y"] = filteredAccel.y;
  doc["z"] = filteredAccel.z;
  doc["noise_level"] = noise_level;
  doc["threshold_used"] = getAdaptiveThreshold();
  doc["device_time"] = tapStartTime;
  doc["duration"] = millis() - tapStartTime;

  String payload;
  serializeJson(doc, payload);

  if (mqtt.connected())
  {
    tapFeed.publish(payload.c_str());
  }

  Serial.println("TAP detected at " + timestampStr);
  Serial.printf("Duration: %lums, Magnitude: %.3f, Derivative: %.3f\n",
                millis() - tapStartTime, filteredAccel.magnitude, filteredAccel.derivative);
}

void calibrateBaseline()
{
  Serial.println("Calibrating baseline...");
  float sum = 0;
  int samples = 200; // More samples for better baseline

  for (int i = 0; i < samples; i++)
  {
    sensors_event_t event;
    accel.getEvent(&event);
    float magnitude = sqrt(pow(event.acceleration.x / 9.81, 2) +
                           pow(event.acceleration.y / 9.81, 2) +
                           pow(event.acceleration.z / 9.81, 2));
    sum += magnitude;
    delay(5);
  }

  baseline_magnitude = sum / samples;
  Serial.printf("Baseline magnitude: %.3f g\n", baseline_magnitude);
}

void initializeNoiseBuffer()
{
  Serial.println("Initializing noise buffer...");

  // Fill noise buffer with initial readings
  for (int i = 0; i < NOISE_BUFFER_SIZE; i++)
  {
    sensors_event_t event;
    accel.getEvent(&event);
    float magnitude = sqrt(pow(event.acceleration.x / 9.81, 2) +
                           pow(event.acceleration.y / 9.81, 2) +
                           pow(event.acceleration.z / 9.81, 2));
    float deviation = abs(magnitude - baseline_magnitude);
    noiseBuffer[i] = deviation;
    delay(5);
  }

  noiseBufferFull = true;
  noiseBufferIndex = 0;
  updateNoiseLevel();

  Serial.printf("Initial noise level: %.4f g\n", noise_level);
}

String getTimestamp()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return String(millis()); // Fallback to millis if NTP fails
  }

  char buffer[64];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  return String(buffer);
}