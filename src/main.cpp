#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <LiquidCrystal_I2C.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Wire.h>

#include <OneWire.h>

#include "config.h"

namespace {

struct DriveCommand {
  float strafe = 0.0f;
  float surge = 0.0f;
  float yaw = 0.0f;
  bool enabled = false;
  bool heartbeat = false;
  uint32_t updatedAt = 0;
};

struct SensorSnapshot {
  float temperatureC = NAN;
  float ph = NAN;
  float phVoltage = NAN;
  float rollDeg = NAN;
  float pitchDeg = NAN;
  float yawRateDegPerSec = NAN;
  bool imuOk = false;
  bool tempOk = false;
};

struct MotorMix {
  float frontLeft = 0.0f;
  float frontRight = 0.0f;
  float rearLeft = 0.0f;
  float rearRight = 0.0f;
};

LiquidCrystal_I2C lcd(config::LCD_I2C_ADDRESS, config::LCD_COLUMNS, config::LCD_ROWS);
Adafruit_MPU6050 mpu;
OneWire oneWire(config::DS18B20_PIN);
DallasTemperature waterTemperature(&oneWire);
WebServer httpServer(80);
WebSocketsServer webSocket(81);

Servo motorFrontLeft;
Servo motorFrontRight;
Servo motorRearLeft;
Servo motorRearRight;

DriveCommand activeCommand;
SensorSnapshot sensors;
MotorMix currentMix;

bool littleFsReady = false;
bool lcdReady = false;
bool imuReady = false;
bool mdnsReady = false;
bool softApStarted = false;
bool stationConnected = false;
uint32_t lastSensorReadMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastLcdMs = 0;
uint32_t lastTemperatureRequestMs = 0;
uint8_t lcdPage = 0;
bool temperatureRequestPending = false;

float clampUnit(float value) {
  if (value > 1.0f) {
    return 1.0f;
  }
  if (value < -1.0f) {
    return -1.0f;
  }
  return value;
}

float applyDeadbandAndExpo(float input) {
  input = clampUnit(input);
  if (fabs(input) < config::COMMAND_DEADBAND) {
    return 0.0f;
  }

  const float sign = input >= 0.0f ? 1.0f : -1.0f;
  const float magnitude = fabs(input);
  const float blended = ((1.0f - config::INPUT_EXPO) * magnitude) +
                        (config::INPUT_EXPO * magnitude * magnitude * magnitude);
  return sign * blended;
}

float readAveragedPhVoltage() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < config::PH_SAMPLE_COUNT; ++i) {
    sum += analogRead(config::PH_SENSOR_PIN);
    delay(3);
  }

  const float average = static_cast<float>(sum) / config::PH_SAMPLE_COUNT;
  return (average / config::ADC_MAX_READING) * config::ADC_REFERENCE_VOLTAGE;
}

float phFromVoltage(float voltage) {
  const float value = 7.0f + ((config::PH_NEUTRAL_VOLTAGE - voltage) / config::PH_VOLTS_PER_PH);
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 14.0f) {
    return 14.0f;
  }
  return value;
}

void writeMotorSignal(Servo& esc, float command, bool invert) {
  float normalized = clampUnit(command);
  if (invert) {
    normalized *= -1.0f;
  }

  const int deltaRange = normalized >= 0.0f
                             ? static_cast<int>(config::ESC_MAX_US - config::ESC_NEUTRAL_US)
                             : static_cast<int>(config::ESC_NEUTRAL_US - config::ESC_MIN_US);

  const int outputUs = static_cast<int>(config::ESC_NEUTRAL_US + (normalized * deltaRange));
  esc.writeMicroseconds(outputUs);
}

void stopAllMotors() {
  currentMix = {};
  writeMotorSignal(motorFrontLeft, 0.0f, config::INVERT_FRONT_LEFT);
  writeMotorSignal(motorFrontRight, 0.0f, config::INVERT_FRONT_RIGHT);
  writeMotorSignal(motorRearLeft, 0.0f, config::INVERT_REAR_LEFT);
  writeMotorSignal(motorRearRight, 0.0f, config::INVERT_REAR_RIGHT);
}

MotorMix calculateXDriveMix(float strafe, float surge, float yaw) {
  // X-drive mixing for four thrusters mounted at 45 degrees:
  // FL = +surge + strafe + yaw
  // FR = +surge - strafe - yaw
  // RL = +surge - strafe + yaw
  // RR = +surge + strafe - yaw
  MotorMix mix;
  mix.frontLeft = surge + strafe + yaw;
  mix.frontRight = surge - strafe - yaw;
  mix.rearLeft = surge - strafe + yaw;
  mix.rearRight = surge + strafe - yaw;

  const float maxMagnitude = max(
      max(fabs(mix.frontLeft), fabs(mix.frontRight)),
      max(fabs(mix.rearLeft), fabs(mix.rearRight)));

  if (maxMagnitude > 1.0f) {
    mix.frontLeft /= maxMagnitude;
    mix.frontRight /= maxMagnitude;
    mix.rearLeft /= maxMagnitude;
    mix.rearRight /= maxMagnitude;
  }

  return mix;
}

void applyDriveCommand() {
  const uint32_t now = millis();
  if (!activeCommand.enabled || (now - activeCommand.updatedAt) > config::COMMAND_TIMEOUT_MS) {
    stopAllMotors();
    return;
  }

  const float strafe = applyDeadbandAndExpo(activeCommand.strafe) * config::MAX_TRANSLATION;
  const float surge = applyDeadbandAndExpo(activeCommand.surge) * config::MAX_TRANSLATION;
  const float yaw = applyDeadbandAndExpo(activeCommand.yaw) * config::MAX_YAW;

  currentMix = calculateXDriveMix(strafe, surge, yaw);

  writeMotorSignal(motorFrontLeft, currentMix.frontLeft, config::INVERT_FRONT_LEFT);
  writeMotorSignal(motorFrontRight, currentMix.frontRight, config::INVERT_FRONT_RIGHT);
  writeMotorSignal(motorRearLeft, currentMix.rearLeft, config::INVERT_REAR_LEFT);
  writeMotorSignal(motorRearRight, currentMix.rearRight, config::INVERT_REAR_RIGHT);
}

void updateLcd() {
  if (!lcdReady) {
    return;
  }

  lcd.clear();
  if (lcdPage == 0) {
    char line1[17];
    char line2[17];
    snprintf(line1, sizeof(line1), "T:%5.1fC pH:%4.2f", sensors.temperatureC, sensors.ph);
    snprintf(line2, sizeof(line2), "R:%4.0f P:%4.0f", sensors.rollDeg, sensors.pitchDeg);
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
  } else {
    const IPAddress ip = stationConnected ? WiFi.localIP() : WiFi.softAPIP();
    char line1[17];
    char line2[17];
    snprintf(line1, sizeof(line1), "%3d.%3d.%3d.%3d", ip[0], ip[1], ip[2], ip[3]);
    snprintf(line2, sizeof(line2), "Y:%+.2f EN:%d", activeCommand.yaw, activeCommand.enabled);
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }

  lcdPage = (lcdPage + 1) % 2;
}

void readSensors() {
  sensors.phVoltage = readAveragedPhVoltage();
  sensors.ph = phFromVoltage(sensors.phVoltage);

  if (imuReady) {
    sensors_event_t accelEvent;
    sensors_event_t gyroEvent;
    sensors_event_t tempEvent;
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    sensors.rollDeg = atan2(accelEvent.acceleration.y, accelEvent.acceleration.z) * 57.2958f;
    sensors.pitchDeg = atan2(-accelEvent.acceleration.x,
                             sqrt((accelEvent.acceleration.y * accelEvent.acceleration.y) +
                                  (accelEvent.acceleration.z * accelEvent.acceleration.z))) *
                       57.2958f;
    sensors.yawRateDegPerSec = gyroEvent.gyro.z * 57.2958f;
    sensors.imuOk = true;
  } else {
    sensors.rollDeg = NAN;
    sensors.pitchDeg = NAN;
    sensors.yawRateDegPerSec = NAN;
    sensors.imuOk = false;
  }
}

void updateTemperatureSensor() {
  const uint32_t now = millis();

  if (temperatureRequestPending &&
      (now - lastTemperatureRequestMs) >= config::TEMPERATURE_CONVERSION_MS) {
    const float temp = waterTemperature.getTempCByIndex(0);
    sensors.tempOk = temp > -100.0f && temp < 125.0f;
    sensors.temperatureC = sensors.tempOk ? temp : NAN;
    temperatureRequestPending = false;
  }

  if (!temperatureRequestPending &&
      (now - lastTemperatureRequestMs) >= config::TEMPERATURE_REQUEST_INTERVAL_MS) {
    waterTemperature.requestTemperatures();
    lastTemperatureRequestMs = now;
    temperatureRequestPending = true;
  }
}

String telemetryJson() {
  JsonDocument doc;
  doc["type"] = "telemetry";
  doc["device"] = config::DEVICE_NAME;
  doc["uptimeMs"] = millis();
  doc["mode"] = stationConnected ? "station" : "access-point";
  doc["ip"] = stationConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  doc["rssi"] = stationConnected ? WiFi.RSSI() : 0;
  doc["tempC"] = sensors.temperatureC;
  doc["ph"] = sensors.ph;
  doc["phVoltage"] = sensors.phVoltage;
  doc["roll"] = sensors.rollDeg;
  doc["pitch"] = sensors.pitchDeg;
  doc["yawRate"] = sensors.yawRateDegPerSec;
  doc["imuOk"] = sensors.imuOk;
  doc["tempOk"] = sensors.tempOk;
  doc["enabled"] = activeCommand.enabled;

  JsonObject motors = doc["motors"].to<JsonObject>();
  motors["frontLeft"] = currentMix.frontLeft;
  motors["frontRight"] = currentMix.frontRight;
  motors["rearLeft"] = currentMix.rearLeft;
  motors["rearRight"] = currentMix.rearRight;

  String payload;
  serializeJson(doc, payload);
  return payload;
}

void broadcastTelemetry() {
  String payload = telemetryJson();
  webSocket.broadcastTXT(payload);
}

void handleDriveMessage(const JsonDocument& doc) {
  activeCommand.strafe = clampUnit(doc["x"] | 0.0f);
  activeCommand.surge = clampUnit(doc["y"] | 0.0f);
  activeCommand.yaw = clampUnit(doc["yaw"] | 0.0f);
  activeCommand.enabled = doc["enabled"] | false;
  activeCommand.heartbeat = doc["heartbeat"] | false;
  activeCommand.updatedAt = millis();
}

void handleSocketText(uint8_t clientId, const String& payload) {
  JsonDocument doc;
  const DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    webSocket.sendTXT(clientId, "{\"type\":\"error\",\"message\":\"invalid-json\"}");
    return;
  }

  const String messageType = doc["type"] | "";
  if (messageType == "drive") {
    handleDriveMessage(doc);
    return;
  }

  if (messageType == "ping") {
    webSocket.sendTXT(clientId, "{\"type\":\"pong\"}");
  }
}

void webSocketEvent(uint8_t clientId, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      String initialPayload = telemetryJson();
      webSocket.sendTXT(clientId, initialPayload);
      break;
    }

    case WStype_TEXT: {
      String message;
      message.reserve(length);
      for (size_t i = 0; i < length; ++i) {
        message += static_cast<char>(payload[i]);
      }
      handleSocketText(clientId, message);
      break;
    }

    case WStype_DISCONNECTED:
      activeCommand.enabled = false;
      stopAllMotors();
      break;

    default:
      break;
  }
}

void serveDashboard() {
  if (littleFsReady && LittleFS.exists("/index.html")) {
    File file = LittleFS.open("/index.html", "r");
    httpServer.streamFile(file, "text/html; charset=utf-8");
    file.close();
    return;
  }

  httpServer.send(200, "text/plain", "Dashboard file missing");
}

void serveConfig() {
  JsonDocument doc;
  doc["device"] = config::DEVICE_NAME;
  doc["commandTimeoutMs"] = config::COMMAND_TIMEOUT_MS;
  doc["axisStrafe"] = config::AXIS_STRAFE;
  doc["axisSurge"] = config::AXIS_SURGE;
  doc["axisYaw"] = config::AXIS_YAW;

  String payload;
  serializeJson(doc, payload);
  httpServer.send(200, "application/json", payload);
}

void setupHttpServer() {
  httpServer.on("/", HTTP_GET, serveDashboard);
  httpServer.on("/api/config", HTTP_GET, serveConfig);
  httpServer.on("/health", HTTP_GET, []() { httpServer.send(200, "text/plain", "ok"); });
  httpServer.onNotFound([]() {
    if (littleFsReady && LittleFS.exists(httpServer.uri())) {
      File file = LittleFS.open(httpServer.uri(), "r");
      httpServer.streamFile(file, "text/plain");
      file.close();
      return;
    }

    httpServer.send(404, "text/plain", "Not found");
  });
  httpServer.begin();
}

void connectStationMode() {
  if (strlen(WIFI_SSID) == 0) {
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < config::WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
  }

  stationConnected = WiFi.status() == WL_CONNECTED;
}

void startAccessPoint() {
  WiFi.mode(stationConnected ? WIFI_AP_STA : WIFI_AP);
  softApStarted = WiFi.softAP(config::ACCESS_POINT_SSID, config::ACCESS_POINT_PASSWORD);
}

void setupWiFi() {
  WiFi.setSleep(false);
  connectStationMode();
  startAccessPoint();

  mdnsReady = MDNS.begin(config::DEVICE_NAME);
  if (mdnsReady) {
    MDNS.addService("http", "tcp", 80);
  }
}

void setupMotorOutputs() {
  motorFrontLeft.setPeriodHertz(50);
  motorFrontRight.setPeriodHertz(50);
  motorRearLeft.setPeriodHertz(50);
  motorRearRight.setPeriodHertz(50);

  motorFrontLeft.attach(config::MOTOR_FRONT_LEFT_PIN, config::ESC_MIN_US, config::ESC_MAX_US);
  motorFrontRight.attach(config::MOTOR_FRONT_RIGHT_PIN, config::ESC_MIN_US, config::ESC_MAX_US);
  motorRearLeft.attach(config::MOTOR_REAR_LEFT_PIN, config::ESC_MIN_US, config::ESC_MAX_US);
  motorRearRight.attach(config::MOTOR_REAR_RIGHT_PIN, config::ESC_MIN_US, config::ESC_MAX_US);

  stopAllMotors();
  delay(config::ESC_ARM_DELAY_MS);
}

void setupPeripherals() {
  Wire.begin(config::I2C_SDA_PIN, config::I2C_SCL_PIN);

  lcd.init();
  lcd.backlight();
  lcdReady = true;
  lcd.setCursor(0, 0);
  lcd.print("BlueSentinel");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");

  imuReady = mpu.begin();
  if (imuReady) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  waterTemperature.begin();
  waterTemperature.setResolution(10);
  waterTemperature.setWaitForConversion(false);
  waterTemperature.requestTemperatures();
  lastTemperatureRequestMs = millis();
  temperatureRequestPending = true;
  analogReadResolution(12);
  analogSetPinAttenuation(config::PH_SENSOR_PIN, ADC_11db);
}

void setupFilesystem() {
  littleFsReady = LittleFS.begin(true);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(config::STATUS_LED_PIN, OUTPUT);
  digitalWrite(config::STATUS_LED_PIN, config::STATUS_LED_ACTIVE_HIGH ? HIGH : LOW);

  setupPeripherals();
  setupFilesystem();
  setupMotorOutputs();
  setupWiFi();
  setupHttpServer();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  lastSensorReadMs = millis();
  lastTelemetryMs = millis();
  lastLcdMs = millis();
  readSensors();
  updateLcd();
}

void loop() {
  httpServer.handleClient();
  webSocket.loop();
  stationConnected = WiFi.status() == WL_CONNECTED;

  const uint32_t now = millis();

  if ((now - lastSensorReadMs) >= config::SENSOR_INTERVAL_MS) {
    lastSensorReadMs = now;
    readSensors();
  }

  updateTemperatureSensor();
  applyDriveCommand();

  if ((now - lastTelemetryMs) >= config::TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    broadcastTelemetry();
  }

  if ((now - lastLcdMs) >= config::LCD_INTERVAL_MS) {
    lastLcdMs = now;
    updateLcd();
  }
}
