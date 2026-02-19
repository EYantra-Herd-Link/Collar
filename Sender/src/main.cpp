#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <driver/i2s.h>
#include <esp_task_wdt.h>

#define I2S_SCK D0
#define I2S_SD D1
#define I2S_WS D2
#define I2S_PORT I2S_NUM_0

#define PIN_ONE_WIRE D3
#define PIN_I2C_SDA D4
#define PIN_I2C_SCL D5
#define PIN_LED D6
#define PIN_MPU_INT D10

#define THI_THRESHOLD_MILD 80.0
#define THI_THRESHOLD_MODERATE 84.0
#define THI_THRESHOLD_SEVERE 88.0

#define SAMPLE_INTERVAL_SECONDS 10
#define REPORT_INTERVAL_SECONDS 10
#define SAMPLES_PER_REPORT (REPORT_INTERVAL_SECONDS / SAMPLE_INTERVAL_SECONDS)

#define uS_TO_S_FACTOR 1000000ULL
#define WATCHDOG_TIMEOUT_S 120

#define ACTIVITY_MIN_THRESHOLD 1.0
#define ACTIVITY_RESTING 10.0
#define ACTIVITY_GRAZING 40.0
#define ACTIVITY_WALKING 100.0

#define AUDIO_SAMPLE_RATE 16000
#define SAMPLES_PER_ENV_POINT 800
#define ENVELOPE_SIZE 64
#define ENVELOPE_SAMPLE_RATE 20.0f

#define RUMINATION_CHEW_FREQ_MIN 0.7
#define RUMINATION_CHEW_FREQ_MAX 1.3
#define RUMINATION_GYRO_THRESHOLD 15.0
#define RUMINATION_AUDIO_THRESHOLD 1000.0

#define MIN_DAILY_RUMINATION_HOURS 6.0
#define MAX_DAILY_RUMINATION_HOURS 10.0
#define MIN_DAILY_GRAZING_HOURS 4.0
#define MAX_DAILY_GRAZING_HOURS 7.0

OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature ds18b20(&oneWire);
Adafruit_AHTX0 aht20;
Adafruit_MPU6050 mpu;
WiFiUDP udp;

const char* WIFI_SSID = "Aditya";
const char* WIFI_PASSWORD = "aditya@123";
const int UDP_PORT = 4210;
const char* TARGET_IP = "255.255.255.255";

float cosTable[ENVELOPE_SIZE];
float sinTable[ENVELOPE_SIZE];

int currentSample = 0;

float sumBodyTemp = 0.0;
float sumEnvTemp = 0.0;
float sumEnvHum = 0.0;
int validTempSamples = 0;

unsigned long dailyRuminationSeconds = 0;
unsigned long dailyGrazingSeconds = 0;
unsigned long dailyRestingSeconds = 0;
unsigned long dailyWalkingSeconds = 0;
unsigned long dailyAlertSeconds = 0;
unsigned long dailyAlertCount = 0;

unsigned long bootTime = 0;
unsigned long lastMidnightReset = 0;

enum StressLevel {
  NORMAL,
  MILD,
  MODERATE,
  SEVERE,
  SENSOR_ERROR
};

enum BehaviorState {
  RUMINATING,
  GRAZING,
  RESTING,
  WALKING,
  ALERT_HIGH_ACTIVITY,
  ALERT_LOW_ACTIVITY,
  UNKNOWN
};

struct SensorHealth {
  bool ds18b20_ok = false;
  bool aht20_ok = false;
  bool mpu6050_ok = false;
  bool mic_ok = false;
  int error_count = 0;
};

SensorHealth sensorHealth;

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  
  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result == ESP_OK) {
    i2s_set_pin(I2S_PORT, &pin_config);
    sensorHealth.mic_ok = true;
    Serial.println("I2S Microphone Initialized");
  } else {
    sensorHealth.mic_ok = false;
    Serial.println("I2S Microphone Failed");
  }
}

void initFFTTables() {
  for (int i = 0; i < ENVELOPE_SIZE; i++) {
    cosTable[i] = cos(2.0 * PI * i / ENVELOPE_SIZE);
    sinTable[i] = sin(2.0 * PI * i / ENVELOPE_SIZE);
  }
}

void computeEnvelopeFFT(float* data, float* magnitudes) {
  for (int k = 0; k < ENVELOPE_SIZE/2; k++) {
    float real = 0;
    float imag = 0;
    
    for (int t = 0; t < ENVELOPE_SIZE; t++) {
      int idx = (k * t) % ENVELOPE_SIZE;
      real += data[t] * cosTable[idx];
      imag += data[t] * sinTable[idx];
    }
    
    magnitudes[k] = sqrt(real*real + imag*imag);
  }
}

struct AudioAnalysis {
  float avgVolume;
  float dominantFreq;
  bool hasRhythmicPattern;
  bool likelyRumination;
};

AudioAnalysis analyzeAudio() {
  AudioAnalysis result = {0, 0, false, false};
  
  if (!sensorHealth.mic_ok) return result;
  
  float envelope[ENVELOPE_SIZE];
  int32_t raw_buffer[SAMPLES_PER_ENV_POINT];
  double total_session_volume = 0;
  
  size_t bytes_read = 0;
  
  // Clear buffer
  i2s_read(I2S_PORT, &raw_buffer, 1024, &bytes_read, 100);

  for (int i = 0; i < ENVELOPE_SIZE; i++) {
    esp_err_t read_result = i2s_read(I2S_PORT, &raw_buffer, sizeof(raw_buffer), &bytes_read, portMAX_DELAY);
    
    if (read_result != ESP_OK) {
        sensorHealth.mic_ok = false;
        return result;
    }
    
    int num_samples = bytes_read / sizeof(int32_t);
    double chunk_sum = 0;
    
    for(int j=0; j<num_samples; j++) {
        chunk_sum += abs(raw_buffer[j]);
    }
    
    float chunk_avg = (num_samples > 0) ? (chunk_sum / num_samples) : 0;
    envelope[i] = chunk_avg;
    total_session_volume += chunk_avg;
  }
  
  result.avgVolume = total_session_volume / ENVELOPE_SIZE;
  
  for (int i = 0; i < ENVELOPE_SIZE; i++) {
    envelope[i] -= result.avgVolume;
  }
  
  float magnitudes[ENVELOPE_SIZE/2];
  computeEnvelopeFFT(envelope, magnitudes);
  
  float maxMag = 0;
  int maxIdx = 0;
  
  for (int i = 2; i <= 5; i++) {
    if (magnitudes[i] > maxMag) {
      maxMag = magnitudes[i];
      maxIdx = i;
    }
  }
  
  result.dominantFreq = (float)maxIdx * ENVELOPE_SAMPLE_RATE / ENVELOPE_SIZE;
  
  bool freqInRange = (result.dominantFreq >= RUMINATION_CHEW_FREQ_MIN && 
                      result.dominantFreq <= RUMINATION_CHEW_FREQ_MAX);
                      
  result.hasRhythmicPattern = (freqInRange && maxMag > RUMINATION_AUDIO_THRESHOLD);
  result.likelyRumination = result.hasRhythmicPattern;
  
  return result;
}

struct MotionAnalysis {
  float activityIntensity;
  float gyroVariance;
  bool hasRhythmicHead;
  bool hasHorizontalMovement;
};

MotionAnalysis analyzeMotion() {
  MotionAnalysis result = {0, 0, false, false};
  
  if (!sensorHealth.mpu6050_ok) return result;
  
  mpu.enableSleep(false);
  delay(10);
  
  float totalJerk = 0.0;
  float prevMag = 0.0;
  float gyroPitch[10];
  float gyroYaw[10];
  int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    if (!mpu.getEvent(&a, &g, &temp)) {
      sensorHealth.mpu6050_ok = false;
      mpu.enableSleep(true);
      return result;
    }
    
    float mag = sqrt(a.acceleration.x * a.acceleration.x + 
                     a.acceleration.y * a.acceleration.y + 
                     a.acceleration.z * a.acceleration.z);
    
    if (i > 0) {
      totalJerk += abs(mag - prevMag);
    }
    prevMag = mag;
    
    gyroPitch[i] = g.gyro.y;
    gyroYaw[i] = g.gyro.z;
    
    delay(100);
  }
  
  result.activityIntensity = totalJerk;
  
  float meanPitch = 0;
  for (int i = 0; i < samples; i++) meanPitch += gyroPitch[i];
  meanPitch /= samples;
  
  float variance = 0;
  for (int i = 0; i < samples; i++) {
    float diff = gyroPitch[i] - meanPitch;
    variance += diff * diff;
  }
  variance /= samples;
  result.gyroVariance = variance;
  
  result.hasRhythmicHead = (variance > RUMINATION_GYRO_THRESHOLD && 
                             result.activityIntensity < ACTIVITY_RESTING);
  
  float meanYaw = 0;
  for (int i = 0; i < samples; i++) meanYaw += abs(gyroYaw[i]);
  meanYaw /= samples;
  result.hasHorizontalMovement = (meanYaw > 20.0);
  
  mpu.enableSleep(true);
  return result;
}

BehaviorState classifyBehavior(MotionAnalysis motion, AudioAnalysis audio) {
  float activity = motion.activityIntensity;
  
  if (activity < ACTIVITY_MIN_THRESHOLD) {
    return ALERT_LOW_ACTIVITY;
  }
  
  if (activity > ACTIVITY_WALKING * 2) {
    return ALERT_HIGH_ACTIVITY;
  }
  
  if (activity < ACTIVITY_RESTING) {
    if (audio.likelyRumination || motion.hasRhythmicHead) {
      return RUMINATING;
    }
    return RESTING;
  }
  
  if (activity < ACTIVITY_GRAZING) {
    if (motion.hasHorizontalMovement) {
      return GRAZING;
    }
    return RESTING;
  }
  
  if (activity < ACTIVITY_WALKING) {
    return WALKING;
  }
  
  return UNKNOWN;
}

String getBehaviorString(BehaviorState state) {
  switch (state) {
    case RUMINATING: return "Ruminating";
    case GRAZING: return "Grazing";
    case RESTING: return "Resting";
    case WALKING: return "Walking";
    case ALERT_HIGH_ACTIVITY: return "ALERT_HighActivity";
    case ALERT_LOW_ACTIVITY: return "ALERT_LowActivity";
    default: return "Unknown";
  }
}

void blinkLed(int duration) {
  digitalWrite(PIN_LED, LOW);
  delay(duration);
  digitalWrite(PIN_LED, HIGH);
}

void indicateBehavior(BehaviorState state) {
  switch (state) {
    case RUMINATING:
      blinkLed(100); delay(100); blinkLed(100);
      break;
    case GRAZING:
      blinkLed(200);
      break;
    case ALERT_HIGH_ACTIVITY:
    case ALERT_LOW_ACTIVITY:
      for (int i = 0; i < 5; i++) { blinkLed(50); delay(50); }
      break;
    default:
      blinkLed(50);
  }
}

float calculateTHI(float t, float rh) {
  return (0.8 * t) + ((rh / 100.0) * (t - 14.4)) + 46.4;
}

StressLevel getStressLevel(float thi) {
  if (thi < THI_THRESHOLD_MILD) return NORMAL;
  if (thi < THI_THRESHOLD_MODERATE) return MILD;
  if (thi < THI_THRESHOLD_SEVERE) return MODERATE;
  return SEVERE;
}

String getStressLevelString(StressLevel level) {
  switch (level) {
    case NORMAL: return "Normal";
    case MILD: return "Mild";
    case MODERATE: return "Moderate";
    case SEVERE: return "Severe";
    default: return "Error";
  }
}

void sendReport(BehaviorState currentState) {
  if (validTempSamples == 0) {
    Serial.println("No valid data to report");
    currentSample = 0;
    return;
  }
  
  float avgBody = sumBodyTemp / validTempSamples;
  float avgEnv = sumEnvTemp / validTempSamples;
  float avgHum = sumEnvHum / validTempSamples;
  float avgTHI = calculateTHI(avgEnv, avgHum);
  
  float rumHours = dailyRuminationSeconds / 3600.0;
  float grazHours = dailyGrazingSeconds / 3600.0;
  float restHours = dailyRestingSeconds / 3600.0;
  float walkHours = dailyWalkingSeconds / 3600.0;
  float alertMinutes = dailyAlertSeconds / 60.0;
  
  unsigned long uptimeHours = (millis() - bootTime) / 3600000;
  unsigned long millisSinceMidnight = millis() - lastMidnightReset;
  unsigned long hoursSinceMidnight = millisSinceMidnight / 3600000;
  unsigned long minutesSinceMidnight = (millisSinceMidnight % 3600000) / 60000;
  
  String alertType = "None";
  StressLevel stress = getStressLevel(avgTHI);
  bool thiAlert = (stress == MODERATE || stress == SEVERE);
  bool moveAlert = (currentState == ALERT_HIGH_ACTIVITY || currentState == ALERT_LOW_ACTIVITY);
  
  if (thiAlert && moveAlert) {
    if (currentState == ALERT_HIGH_ACTIVITY) alertType = "High THI, Abnormal Movement";
    else alertType = "High THI, Abnormal Stillness";
  } else if (thiAlert) {
    alertType = "High THI";
  } else if (moveAlert) {
    if (currentState == ALERT_HIGH_ACTIVITY) alertType = "Abnormal Movement";
    else alertType = "Abnormal Stillness";
  }

  Serial.println("\nCUMULATIVE DAILY REPORT");
  Serial.print("Time Since Midnight: "); 
  Serial.print(hoursSinceMidnight); 
  Serial.print("h "); 
  Serial.print(minutesSinceMidnight); 
  Serial.println("m");
  
  Serial.print("Body Temperature: "); Serial.print(avgBody, 2); Serial.println(" C");
  Serial.print("Ambient Temp: "); Serial.print(avgEnv, 2); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(avgHum, 1); Serial.println(" %");
  Serial.print("THI Stress: "); Serial.println(getStressLevelString(stress));
  
  Serial.println("\nCUMULATIVE TIME TODAY:");
  Serial.print("  Rumination: "); Serial.print(rumHours, 2); Serial.print(" hours");
  if (uptimeHours >= 6 && hoursSinceMidnight >= 6) {
    if (rumHours < MIN_DAILY_RUMINATION_HOURS) Serial.print(" [LOW WARNING]");
    else if (rumHours > MAX_DAILY_RUMINATION_HOURS) Serial.print(" [HIGH WARNING]");
    else Serial.print(" [NORMAL]");
  }
  Serial.println();
  
  Serial.print("  Grazing: "); Serial.print(grazHours, 2); Serial.print(" hours");
  if (uptimeHours >= 6 && hoursSinceMidnight >= 6) {
    if (grazHours < MIN_DAILY_GRAZING_HOURS) Serial.print(" [LOW WARNING]");
    else if (grazHours > MAX_DAILY_GRAZING_HOURS) Serial.print(" [HIGH WARNING]");
    else Serial.print(" [NORMAL]");
  }
  Serial.println();
  
  Serial.print("  Resting: "); Serial.print(restHours, 2); Serial.println(" hours");
  Serial.print("  Walking: "); Serial.print(walkHours, 2); Serial.println(" hours");
  if (dailyAlertCount > 0) {
    Serial.print("  Alerts Count: "); Serial.println(dailyAlertCount);
    Serial.print("  Last Alert Type: "); Serial.println(alertType);
  }
  
  Serial.print("\nSystem Uptime: "); Serial.print(uptimeHours); Serial.println(" hours");
  Serial.print("Sensor Errors: "); Serial.println(sensorHealth.error_count);
  Serial.println();
  Serial.flush();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi Connected. IP: ");
    Serial.println(WiFi.localIP().toString());
    
    udp.begin(UDP_PORT);
    
    String payload = "{";
    payload += "\"timestamp_hours\":" + String(hoursSinceMidnight) + ",";
    payload += "\"body_temp\":" + String(avgBody, 2) + ",";
    payload += "\"env_temp\":" + String(avgEnv, 2) + ",";
    payload += "\"humidity\":" + String(avgHum, 1) + ",";
    payload += "\"thi_stress\":\"" + getStressLevelString(stress) + "\",";
    payload += "\"rumination_hours\":" + String(rumHours, 2) + ",";
    payload += "\"grazing_hours\":" + String(grazHours, 2) + ",";
    payload += "\"resting_hours\":" + String(restHours, 2) + ",";
    payload += "\"walking_hours\":" + String(walkHours, 2) + ",";
    payload += "\"alert_type\":\"" + alertType + "\",";
    payload += "\"alert_count\":" + String(dailyAlertCount) + ",";
    payload += "\"current_activity\":\"" + getBehaviorString(currentState) + "\",";
    payload += "\"errors\":" + String(sensorHealth.error_count);
    payload += "}";

    udp.beginPacket(TARGET_IP, UDP_PORT);
    udp.print(payload);
    udp.endPacket();
  } else {
      Serial.println("\nWiFi Connection failed.");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  // Default LED off (usually HIGH is off or LOW is off depending on board, assumes LOW is ON based on blinkLed?)
  // blinkLed does LOW then HIGH. If HIGH is OFF, then default HIGH.
  digitalWrite(PIN_LED, HIGH);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize FFT
  initFFTTables();

  // Sensors
  ds18b20.begin();
  if (ds18b20.getDeviceCount() > 0) {
      sensorHealth.ds18b20_ok = true;
      Serial.println("DS18B20 found");
  } else {
      Serial.println("DS18B20 not found");
  }

  if (aht20.begin()) {
      sensorHealth.aht20_ok = true;
      Serial.println("AHT20 found");
  } else {
      Serial.println("AHT20 not found");
  }

  if (mpu.begin()) {
    sensorHealth.mpu6050_ok = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 found");
  } else {
    Serial.println("MPU6050 not found");
  }

  setupI2S();

  // Initial WiFi attempt (optional, since we connect on report)
  // But good to establish connection early if possible.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  bootTime = millis();
  lastMidnightReset = millis();
}

unsigned long lastSampleTime = 0;
int samplesSinceReport = 0;

void loop() {
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL_SECONDS * 1000) {
    lastSampleTime = millis();
    
    // Read Temps
    ds18b20.requestTemperatures();
    float bodyTemp = ds18b20.getTempCByIndex(0);
    
    sensors_event_t humidity, temp;
    if (sensorHealth.aht20_ok) {
        aht20.getEvent(&humidity, &temp);
        sumEnvTemp += temp.temperature;
        sumEnvHum += humidity.relative_humidity;
    }
    
    if (bodyTemp != DEVICE_DISCONNECTED_C) {
        sumBodyTemp += bodyTemp;
        validTempSamples++;
    }

    // Analyze
    AudioAnalysis audio = analyzeAudio();
    MotionAnalysis motion = analyzeMotion();
    BehaviorState state = classifyBehavior(motion, audio);
    
    Serial.print("State: "); Serial.println(getBehaviorString(state));
    indicateBehavior(state);
    
    // Update counters
    unsigned long interval = SAMPLE_INTERVAL_SECONDS;
    switch(state) {
        case RUMINATING: dailyRuminationSeconds += interval; break;
        case GRAZING: dailyGrazingSeconds += interval; break;
        case RESTING: dailyRestingSeconds += interval; break;
        case WALKING: dailyWalkingSeconds += interval; break;
        case ALERT_HIGH_ACTIVITY: 
        case ALERT_LOW_ACTIVITY: 
            dailyAlertSeconds += interval; 
            dailyAlertCount++;
            break;
        default: break;
    }
    
    samplesSinceReport++;
    
    if (samplesSinceReport >= SAMPLES_PER_REPORT) {
        sendReport(state);
        // Reset accumulators
        sumBodyTemp = 0;
        sumEnvTemp = 0;
        sumEnvHum = 0;
        validTempSamples = 0;
        samplesSinceReport = 0;
    }
  }
  
  // Daily Reset check
  if (millis() - lastMidnightReset > 24 * 3600 * 1000UL) {
      dailyRuminationSeconds = 0;
      dailyGrazingSeconds = 0;
      dailyRestingSeconds = 0;
      dailyWalkingSeconds = 0;
      dailyAlertSeconds = 0;
      dailyAlertCount = 0;
      lastMidnightReset = millis();
      Serial.println("Daily counters reset");
  }
}
