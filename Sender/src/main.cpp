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
#define SENSOR_POLL_RATE_MS         10000
#define IMU_SAMPLES_PER_CYCLE       10
#define IMU_SAMPLE_INTERVAL_MS      100
#define RUMINATION_PITCH_THRESHOLD  0.5f
#define AUDIO_SAMPLE_RATE           16000
#define RUMINATION_AUDIO_THRESHOLD  50000.0f
#define WDT_TIMEOUT_SECONDS         20
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

const char* WIFI_SSID     = "Aditya";
const char* WIFI_PASSWORD = "aditya@123";
const int   UDP_PORT      = 4210;
const char* TARGET_IP     = "255.255.255.255";

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
float             latestAudioMagnitude = 0.0f;

volatile bool motionInterruptFired = false;
static uint32_t seqNum = 0;

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
  BehaviorState state;
  bool          motion_interrupt;
  bool          low_activity_flag;
  bool          ds18b20_ok;
  bool          aht20_ok;
  bool          mpu_ok;
  bool          mic_ok;
  int           errorCount;
  bool          isError;
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

void IRAM_ATTR mpuInterruptHandler() {
  motionInterruptFired = true;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(TaskSensorHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SENSOR_POLL_RATE_MS);

  for (;;) {
    esp_task_wdt_reset();

    bool urgentMotion = false;
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
      urgentMotion = true;
    }

    if (!urgentMotion) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    } else {
      xLastWakeTime = xTaskGetTickCount();
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
        rec.bodyTemp          = 0.0f;
        rec.isError           = true;
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
          rec.isError        = true;
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

      mpu.enableSleep(true);

      if (validSamples > 0) {
        rec.ax = sumAx / validSamples;
        rec.ay = sumAy / validSamples;
        rec.az = sumAz / validSamples;
        rec.gx = sumGx / validSamples;
        rec.gy = sumGy / validSamples;
        rec.gz = sumGz / validSamples;
        meanPitch /= validSamples;

        float mean = rec.gy;
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
        mpu.enableMotionDetection(MPU_MOTION_THRESHOLD);
        mpu.setMotionDetectionDuration(MPU_MOTION_DURATION);
        sensorHealth.mpu_ok = true;
      }
    }

    rec.activityJerk = (validSamples > 1) ? totalJerk / (validSamples - 1) : 0.0f;
    rec.low_activity_flag = (rec.activityJerk < JERK_MIN && validSamples > 1);

    if (urgentMotion) {
      rec.state = ALERT_HIGH_ACTIVITY;
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
    rec.audioMagnitude   = latestAudioMagnitude;
    latestAudioMagnitude = 0.0f;
    xSemaphoreGive(audioMutex);

    if (rec.audioMagnitude > RUMINATION_AUDIO_THRESHOLD) {
      rec.state = RUMINATING;
    }

    rec.ds18b20_ok = sensorHealth.ds18b20_ok;
    rec.aht20_ok   = sensorHealth.aht20_ok;
    rec.mpu_ok     = sensorHealth.mpu_ok;
    rec.mic_ok     = sensorHealth.mic_ok;
    rec.errorCount = sensorHealth.error_count;

    indicateBehavior(rec.state);

    if (xQueueSend(healthDataQueue, &rec, pdMS_TO_TICKS(100)) != pdPASS) {
      Serial.println("Q_FULL");
    }
  }
}

void audioTask(void *pvParameters) {
  esp_task_wdt_add(NULL);

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!sensorHealth.mic_ok) continue;

    esp_task_wdt_reset();
    i2s_start(I2S_PORT);

    size_t  bytes_read = 0;
    int32_t raw_buffer[128];
    i2s_read(I2S_PORT, &raw_buffer, sizeof(raw_buffer), &bytes_read, pdMS_TO_TICKS(100));

    double   sum_squares   = 0.0;
    uint32_t total_samples = 0;
    bool     read_error    = false;

    for (int i = 0; i < 5; i++) {
      esp_task_wdt_reset();
      esp_err_t res = i2s_read(I2S_PORT, &raw_buffer, sizeof(raw_buffer),
                               &bytes_read, pdMS_TO_TICKS(200));
      if (res != ESP_OK) {
        read_error          = true;
        sensorHealth.mic_ok = false;
        sensorHealth.error_count++;
        break;
      }

      int num_samples = bytes_read / sizeof(int32_t);
      for (int j = 0; j < num_samples; j++) {
        double val = (double)raw_buffer[j];
        sum_squares += val * val;
        total_samples++;
      }

      vTaskDelay(pdMS_TO_TICKS(10));
    }

    i2s_stop(I2S_PORT);

    if (!read_error && total_samples > 0) {
      float rms = sqrt(sum_squares / total_samples);
      xSemaphoreTake(audioMutex, portMAX_DELAY);
      latestAudioMagnitude = rms;
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

    char payload[700];
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
      "\"aud\":%.2f,"
      "\"state\":%d,"
      "\"mot_int\":%d,"
      "\"low_act\":%d,"
      "\"s_ds18b20\":%d,"
      "\"s_aht20\":%d,"
      "\"s_mpu\":%d,"
      "\"s_mic\":%d,"
      "\"errcnt\":%d,"
      "\"err\":%d"
      "}",
      device_id,
      rec.seq,
      rec.timestamp,
      rec.bodyTemp,
      rec.envTemp,
      rec.envHum,
      rec.ax, rec.ay, rec.az,
      rec.gx, rec.gy, rec.gz,
      rec.gy_variance,
      rec.activityJerk,
      rec.audioMagnitude,
      (int)rec.state,
      (int)rec.motion_interrupt,
      (int)rec.low_activity_flag,
      (int)rec.ds18b20_ok,
      (int)rec.aht20_ok,
      (int)rec.mpu_ok,
      (int)rec.mic_ok,
      rec.errorCount,
      (int)rec.isError
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
    mpu.enableMotionDetection(MPU_MOTION_THRESHOLD);
    mpu.setMotionDetectionDuration(MPU_MOTION_DURATION);
    pinMode(PIN_MPU_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), mpuInterruptHandler, RISING);
    sensorHealth.mpu_ok = true;
  }

  i2s_config_t i2s_config = {
    .mode               = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate        = AUDIO_SAMPLE_RATE,
    .bits_per_sample    = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format     = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags   = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count      = 4,
    .dma_buf_len        = 512,
    .use_apll           = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk         = 0
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
