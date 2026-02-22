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
#include <esp_mac.h>
#include <esp_pm.h>
#include <esp_sleep.h>

#define I2S_SCK         D0
#define I2S_SD          D1
#define I2S_WS          D2
#define I2S_PORT        I2S_NUM_0
#define PIN_ONE_WIRE    D3
#define PIN_I2C_SDA     D4
#define PIN_I2C_SCL     D5
#define PIN_LED         D6
#define PIN_MPU_INT     D10

#define QUEUE_LENGTH                20
#define SENSOR_POLL_RATE_MS         45000
#define IMU_SAMPLES_PER_CYCLE       10
#define IMU_SAMPLE_INTERVAL_MS      100
#define RUMINATION_PITCH_THRESHOLD  0.5f
#define AUDIO_SAMPLE_RATE           16000
#define RUMINATION_AUDIO_THRESHOLD  50000.0f
#define WDT_TIMEOUT_SECONDS         30
#define WIFI_MAX_RETRIES            10
#define WIFI_RETRY_INTERVAL_MS      500
#define WIFI_BACKOFF_MS             5000
#define IMU_WAKE_SETTLE_MS          10
#define DS18B20_RESOLUTION          9
#define JERK_WALKING                100.0f
#define JERK_GRAZING                40.0f
#define JERK_MIN                    0.5f
#define MPU_MOTION_THRESHOLD        18
#define MPU_MOTION_DURATION         2

#define FFT_SIZE     256
#define NUM_MEL_BINS 8

const char* WIFI_SSID     = "Aditya";
const char* WIFI_PASSWORD = "aditya@123";
const int   UDP_PORT      = 4210;
const char* TARGET_IP     = "255.255.255.255";

IPAddress local_IP(192, 168, 1, 200);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);

struct SensorHealth {
  bool ds18b20_ok  = false;
  bool aht20_ok    = false;
  bool mpu_ok      = false;
  bool mic_ok      = false;
  int  error_count = 0;
};

SensorHealth sensorHealth;
char device_id[18];

SemaphoreHandle_t audioMutex;
float latestAudioMagnitude = 0.0f;
float latestAudioFeatures[NUM_MEL_BINS] = {0};

volatile bool motionInterruptFired = false;
static uint32_t seqNum = 0;

int16_t cos_lut[FFT_SIZE / 2];
int16_t sin_lut[FFT_SIZE / 2];
int16_t window_lut[FFT_SIZE];

enum BehaviorState {
  RUMINATING,
  GRAZING,
  RESTING,
  WALKING,
  ALERT_HIGH_ACTIVITY,
  ALERT_LOW_ACTIVITY,
  UNKNOWN_STATE
};

struct HealthRecord {
  uint32_t      seq;
  uint32_t      timestamp;
  float         bodyTemp;
  float         envTemp;
  float         envHum;
  float         ax, ay, az;
  float         gx, gy, gz;
  float         gy_variance;
  float         activityJerk;
  float         audioMagnitude;
  float         audioFeatures[NUM_MEL_BINS];
  BehaviorState state;
  bool          motion_interrupt;
  bool          low_activity_flag;
  bool          ds18b20_ok;
  bool          aht20_ok;
  bool          mpu_ok;
  bool          mic_ok;
  int           errorCount;
  bool          isError;
  int           rssi;
};

QueueHandle_t healthDataQueue;
TaskHandle_t  TaskSensorHandle;
TaskHandle_t  TaskAudioHandle;
TaskHandle_t  TaskTelemetryHandle;

OneWire           oneWire(PIN_ONE_WIRE);
DallasTemperature ds18b20(&oneWire);
Adafruit_AHTX0    aht20;
Adafruit_MPU6050  mpu;
WiFiUDP           udp;

void initDSP() {
  for (int i = 0; i < FFT_SIZE / 2; i++) {
    cos_lut[i] = (int16_t)(cos(2.0 * PI * i / FFT_SIZE) * 32767.0);
    sin_lut[i] = (int16_t)(sin(2.0 * PI * i / FFT_SIZE) * 32767.0);
  }
  for (int i = 0; i < FFT_SIZE; i++) {
    window_lut[i] = (int16_t)((0.54 - 0.46 * cos(2.0 * PI * i / (FFT_SIZE - 1))) * 32767.0);
  }
}

void int_fft(int16_t *fr, int16_t *fi, int n) {
  int m = 0;
  while ((1 << m) < n) m++;
  int j = 0;
  for (int i = 0; i < n - 1; i++) {
    if (i < j) {
      int16_t tr = fr[j]; int16_t ti = fi[j];
      fr[j] = fr[i]; fi[j] = fi[i];
      fr[i] = tr;    fi[i] = ti;
    }
    int k = n >> 1;
    while (k <= j) { j -= k; k >>= 1; }
    j += k;
  }
  for (int l = 1; l <= m; l++) {
    int le  = 1 << l;
    int le2 = le >> 1;
    for (j = 0; j < le2; j++) {
      int      idx = j << (m - l);
      int16_t  wr  = cos_lut[idx];
      int16_t  wi  = -sin_lut[idx];
      for (int i = j; i < n; i += le) {
        int ip    = i + le2;
        int32_t tr = ((int32_t)fr[ip] * wr - (int32_t)fi[ip] * wi) >> 15;
        int32_t ti = ((int32_t)fr[ip] * wi + (int32_t)fi[ip] * wr) >> 15;
        fr[ip] = fr[i] - tr;
        fi[ip] = fi[i] - ti;
        fr[i] += tr;
        fi[i] += ti;
      }
    }
  }
}

uint32_t int_sqrt(uint32_t n) {
  uint32_t root = 0;
  uint32_t bit  = 1UL << 30;
  while (bit > n) bit >>= 2;
  while (bit != 0) {
    if (n >= root + bit) {
      n   -= root + bit;
      root = (root >> 1) + bit;
    } else {
      root >>= 1;
    }
    bit >>= 2;
  }
  return root;
}

void IRAM_ATTR mpuInterruptHandler() {
  motionInterruptFired = true;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(TaskSensorHandle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void indicateBehavior(BehaviorState state) {
  switch (state) {
    case RUMINATING:
      digitalWrite(PIN_LED, LOW); vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(PIN_LED, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(PIN_LED, LOW); vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(PIN_LED, HIGH);
      break;
    case GRAZING:
      digitalWrite(PIN_LED, LOW); vTaskDelay(pdMS_TO_TICKS(200));
      digitalWrite(PIN_LED, HIGH);
      break;
    case ALERT_HIGH_ACTIVITY:
    case ALERT_LOW_ACTIVITY:
      for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_LED, LOW); vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(PIN_LED, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
      }
      break;
    default:
      digitalWrite(PIN_LED, LOW); vTaskDelay(pdMS_TO_TICKS(50));
      digitalWrite(PIN_LED, HIGH);
      break;
  }
}

void sensorTask(void *pvParameters) {
  esp_task_wdt_add(NULL);

  for (;;) {
    esp_task_wdt_reset();

    bool urgentMotion = false;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SENSOR_POLL_RATE_MS)) > 0) {
      urgentMotion = true;
    }

    HealthRecord rec = {};
    rec.seq              = seqNum++;
    rec.timestamp        = millis();
    rec.isError          = false;
    rec.motion_interrupt = urgentMotion;
    motionInterruptFired = false;

    if (sensorHealth.ds18b20_ok) {
      float bt = ds18b20.getTempCByIndex(0);
      ds18b20.requestTemperatures();
      if (bt == DEVICE_DISCONNECTED_C || bt < -50.0f) {
        rec.bodyTemp            = 0.0f;
        rec.isError             = true;
        sensorHealth.ds18b20_ok = false;
        sensorHealth.error_count++;
      } else {
        rec.bodyTemp = bt;
      }
    } else {
      if (ds18b20.getDeviceCount() > 0) {
        ds18b20.setResolution(DS18B20_RESOLUTION);
        ds18b20.requestTemperatures();
        sensorHealth.ds18b20_ok = true;
      }
      rec.bodyTemp = 0.0f;
      rec.isError  = true;
    }

    if (sensorHealth.aht20_ok) {
      sensors_event_t humidity, temp;
      if (aht20.getEvent(&humidity, &temp)) {
        rec.envTemp = temp.temperature;
        rec.envHum  = humidity.relative_humidity;
      } else {
        rec.envTemp           = 0.0f;
        rec.envHum            = 0.0f;
        rec.isError           = true;
        sensorHealth.aht20_ok = false;
        sensorHealth.error_count++;
      }
    } else {
      if (aht20.begin()) {
        sensorHealth.aht20_ok = true;
      }
      rec.envTemp = 0.0f;
      rec.envHum  = 0.0f;
      rec.isError = true;
    }

    float totalJerk  = 0.0f;
    float prevMag    = 0.0f;
    float meanPitch  = 0.0f;
    float sumAx = 0, sumAy = 0, sumAz = 0;
    float sumGx = 0, sumGy = 0, sumGz = 0;
    float gyroYSamples[IMU_SAMPLES_PER_CYCLE];
    int   validSamples = 0;

    if (sensorHealth.mpu_ok) {
      mpu.enableSleep(false);
      vTaskDelay(pdMS_TO_TICKS(IMU_WAKE_SETTLE_MS));

      for (int i = 0; i < IMU_SAMPLES_PER_CYCLE; i++) {
        sensors_event_t a, g, mpuTemp;
        if (!mpu.getEvent(&a, &g, &mpuTemp)) {
          rec.isError         = true;
          sensorHealth.mpu_ok = false;
          sensorHealth.error_count++;
          break;
        }

        sumAx += a.acceleration.x;
        sumAy += a.acceleration.y;
        sumAz += a.acceleration.z;
        sumGx += g.gyro.x;
        sumGy += g.gyro.y;
        sumGz += g.gyro.z;
        gyroYSamples[validSamples] = g.gyro.y;

        float mag = sqrt(a.acceleration.x * a.acceleration.x +
                         a.acceleration.y * a.acceleration.y +
                         a.acceleration.z * a.acceleration.z);
        if (i > 0) totalJerk += abs(mag - prevMag);
        prevMag    = mag;
        meanPitch += g.gyro.y;
        validSamples++;

        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_INTERVAL_MS));
      }

      mpu.setInterruptPinLatch(false);
      mpu.enableSleep(true);

      if (validSamples > 0) {
        rec.ax = sumAx / validSamples;
        rec.ay = sumAy / validSamples;
        rec.az = sumAz / validSamples;
        rec.gx = sumGx / validSamples;
        rec.gy = sumGy / validSamples;
        rec.gz = sumGz / validSamples;
        meanPitch /= validSamples;

        float mean     = rec.gy;
        float variance = 0.0f;
        for (int i = 0; i < validSamples; i++) {
          float diff = gyroYSamples[i] - mean;
          variance += diff * diff;
        }
        rec.gy_variance = variance / validSamples;
      }
    } else {
      if (mpu.begin()) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        mpu.enableSleep(true);
        mpu.setMotionDetectionThreshold(MPU_MOTION_THRESHOLD);
        mpu.setMotionDetectionDuration(MPU_MOTION_DURATION);
        mpu.setInterruptPinLatch(false);
        mpu.setInterruptPinPolarity(true);
        mpu.setMotionInterrupt(true);
        sensorHealth.mpu_ok = true;
      }
    }

    rec.activityJerk      = (validSamples > 1) ? totalJerk / (validSamples - 1) : 0.0f;
    rec.low_activity_flag = (rec.activityJerk < JERK_MIN && validSamples > 1);

    if (urgentMotion) {
      rec.state = ALERT_HIGH_ACTIVITY;
      if (sensorHealth.mic_ok) {
        xTaskNotifyGive(TaskAudioHandle);
      }
    } else if (!sensorHealth.mpu_ok) {
      rec.state = UNKNOWN_STATE;
    } else if (rec.activityJerk > JERK_WALKING * 2.0f) {
      rec.state = ALERT_HIGH_ACTIVITY;
    } else if (rec.activityJerk > JERK_WALKING) {
      rec.state = WALKING;
    } else if (rec.activityJerk > JERK_GRAZING) {
      rec.state = GRAZING;
    } else {
      rec.state = RESTING;
      if (sensorHealth.mic_ok && abs(meanPitch) > RUMINATION_PITCH_THRESHOLD) {
        xTaskNotifyGive(TaskAudioHandle);
      }
    }

    xSemaphoreTake(audioMutex, portMAX_DELAY);
    rec.audioMagnitude = latestAudioMagnitude;
    for (int i = 0; i < NUM_MEL_BINS; i++) {
      rec.audioFeatures[i] = latestAudioFeatures[i];
    }
    latestAudioMagnitude = 0.0f;
    memset(latestAudioFeatures, 0, sizeof(latestAudioFeatures));
    xSemaphoreGive(audioMutex);

    if (rec.audioMagnitude > RUMINATION_AUDIO_THRESHOLD) {
      rec.state = RUMINATING;
    }

    rec.ds18b20_ok = sensorHealth.ds18b20_ok;
    rec.aht20_ok   = sensorHealth.aht20_ok;
    rec.mpu_ok     = sensorHealth.mpu_ok;
    rec.mic_ok     = sensorHealth.mic_ok;
    rec.errorCount = sensorHealth.error_count;
    rec.rssi       = WiFi.RSSI();

    indicateBehavior(rec.state);

    if (xQueueSend(healthDataQueue, &rec, pdMS_TO_TICKS(100)) != pdPASS) {
      Serial.println("Q_FULL");
    }
  }
}

void audioTask(void *pvParameters) {
  esp_task_wdt_add(NULL);

  static int32_t raw_buffer[FFT_SIZE];
  static int16_t fr[FFT_SIZE];
  static int16_t fi[FFT_SIZE];

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!sensorHealth.mic_ok) continue;

    esp_task_wdt_reset();
    i2s_start(I2S_PORT);

    size_t bytes_read = 0;
    i2s_read(I2S_PORT, raw_buffer, sizeof(raw_buffer), &bytes_read, pdMS_TO_TICKS(100));

    uint64_t total_rms_sum                  = 0;
    uint32_t mel_accumulators[NUM_MEL_BINS] = {0};
    int      frames_processed               = 0;
    bool     read_error                     = false;

    for (int i = 0; i < 187; i++) {
      if (i % 10 == 0) esp_task_wdt_reset();

      esp_err_t res = i2s_read(I2S_PORT, raw_buffer, sizeof(raw_buffer),
                               &bytes_read, pdMS_TO_TICKS(200));
      if (res != ESP_OK) {
        read_error          = true;
        sensorHealth.mic_ok = false;
        sensorHealth.error_count++;
        break;
      }

      memset(fi, 0, sizeof(fi));

      for (int j = 0; j < FFT_SIZE; j++) {
        int32_t sample = raw_buffer[j] >> 12;
        fr[j] = (int16_t)((sample * window_lut[j]) >> 15);
        total_rms_sum += (uint64_t)(sample * sample);
      }

      int_fft(fr, fi, FFT_SIZE);

      int bin_width = (FFT_SIZE / 2) / NUM_MEL_BINS;
      for (int k = 0; k < FFT_SIZE / 2; k++) {
        uint32_t mag_sq  = ((uint32_t)(fr[k] * fr[k])) + ((uint32_t)(fi[k] * fi[k]));
        uint32_t mag     = int_sqrt(mag_sq);
        int      mel_idx = k / bin_width;
        if (mel_idx >= NUM_MEL_BINS) mel_idx = NUM_MEL_BINS - 1;
        mel_accumulators[mel_idx] += mag;
      }

      frames_processed++;
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    i2s_stop(I2S_PORT);

    if (!read_error && frames_processed > 0) {
      uint32_t final_rms = int_sqrt(
        (uint32_t)(total_rms_sum / ((uint64_t)frames_processed * FFT_SIZE))
      );

      xSemaphoreTake(audioMutex, portMAX_DELAY);
      latestAudioMagnitude = (float)final_rms;
      for (int i = 0; i < NUM_MEL_BINS; i++) {
        latestAudioFeatures[i] = (float)mel_accumulators[i] / frames_processed;
      }
      xSemaphoreGive(audioMutex);
    }
  }
}

void telemetryTask(void *pvParameters) {
  esp_task_wdt_add(NULL);

  for (;;) {
    esp_task_wdt_reset();

    HealthRecord rec;
    if (xQueueReceive(healthDataQueue, &rec, portMAX_DELAY) != pdPASS) {
      continue;
    }

    if (WiFi.status() != WL_CONNECTED) {
      WiFi.config(local_IP, gateway, subnet, primaryDNS);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      int retries = 0;
      while (WiFi.status() != WL_CONNECTED && retries < WIFI_MAX_RETRIES) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS));
        retries++;
      }

      if (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(WIFI_BACKOFF_MS));
        continue;
      }
    }

    char af_buf[128] = "[";
    for (int i = 0; i < NUM_MEL_BINS; i++) {
      char temp[16];
      snprintf(temp, sizeof(temp), "%.2f%s",
               rec.audioFeatures[i],
               (i == NUM_MEL_BINS - 1) ? "" : ",");
      strlcat(af_buf, temp, sizeof(af_buf));
    }
    strlcat(af_buf, "]", sizeof(af_buf));

    char payload[768];
    snprintf(payload, sizeof(payload),
      "{"
      "\"id\":\"%s\","
      "\"seq\":%u,"
      "\"ts\":%u,"
      "\"bt\":%.2f,"
      "\"et\":%.2f,"
      "\"eh\":%.2f,"
      "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
      "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
      "\"gy_var\":%.4f,"
      "\"jerk\":%.2f,"
      "\"mot_int\":%d,"
      "\"err\":%d,"
      "\"v\":0.00,"
      "\"s\":%d,"
      "\"af\":%s"
      "}",
      device_id, rec.seq, rec.timestamp,
      rec.bodyTemp, rec.envTemp, rec.envHum,
      rec.ax, rec.ay, rec.az,
      rec.gx, rec.gy, rec.gz,
      rec.gy_variance, rec.activityJerk,
      rec.motion_interrupt  ? 1 : 0,
      rec.isError           ? 1 : 0,
      rec.rssi, af_buf
    );

    udp.beginPacket(TARGET_IP, UDP_PORT);
    udp.print(payload);
    udp.endPacket();

    if (uxQueueMessagesWaiting(healthDataQueue) == 0) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  esp_pm_config_esp32c3_t pm_config = {
    .max_freq_mhz = 160,
    .min_freq_mhz = 10,
    .light_sleep_enable = true
  };
  esp_pm_configure(&pm_config);

  initDSP();

  WiFi.mode(WIFI_STA);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(device_id, sizeof(device_id), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  ds18b20.begin();
  if (ds18b20.getDeviceCount() > 0) {
    ds18b20.setResolution(DS18B20_RESOLUTION);
    ds18b20.requestTemperatures();
    sensorHealth.ds18b20_ok = true;
  }

  if (aht20.begin()) {
    sensorHealth.aht20_ok = true;
  }

  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu.enableSleep(true);
    mpu.setMotionDetectionThreshold(MPU_MOTION_THRESHOLD);
    mpu.setMotionDetectionDuration(MPU_MOTION_DURATION);
    mpu.setInterruptPinLatch(false);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    pinMode(PIN_MPU_INT, INPUT);
    gpio_wakeup_enable((gpio_num_t)PIN_MPU_INT, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), mpuInterruptHandler, RISING);
    sensorHealth.mpu_ok = true;
  }

  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = AUDIO_SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 4,
    .dma_buf_len          = 512,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD
  };

  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) == ESP_OK) {
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_stop(I2S_PORT);
    sensorHealth.mic_ok = true;
  }

  audioMutex = xSemaphoreCreateMutex();
  if (audioMutex == NULL) while (1);

  healthDataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(HealthRecord));
  if (healthDataQueue == NULL) while (1);

  xTaskCreate(sensorTask,    "SensorTask",    4096, NULL, 3, &TaskSensorHandle);
  xTaskCreate(telemetryTask, "TelemetryTask", 8192, NULL, 2, &TaskTelemetryHandle);
  xTaskCreate(audioTask,     "AudioTask",     8192, NULL, 1, &TaskAudioHandle);
}

void loop() {
  vTaskDelete(NULL);
}
