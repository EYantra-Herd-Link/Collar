#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_pm.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_AHTX0.h>
#include <driver/i2s.h>
#include <esp_mac.h>

#define PIN_I2C_SDA D4
#define PIN_I2C_SCL D5
#define PIN_LED D6
#define PIN_MPU_INT D10
#define PIN_ONE_WIRE D3

#define I2S_SCK D0
#define I2S_SD D1
#define I2S_WS D2
#define I2S_PORT I2S_NUM_0
#define AUDIO_SAMPLE_RATE 16000
#define FFT_SIZE 256
#define NUM_MEL_BINS 8

#define IMU_SAMPLES_PER_CYCLE 10
#define IMU_SAMPLE_INTERVAL_MS 100
#define JERK_WALKING 30.0f
#define JERK_GRAZING 10.0f
#define JERK_MIN 0.5f

const char* WIFI_SSID     = "H320";
const char* WIFI_PASSWORD = "xbvl7713";
const int   UDP_PORT      = 4210;
const char* TARGET_IP     = "255.255.255.255";

#define SENSOR_POLL_RATE_MS 30000 
#define WDT_TIMEOUT_SECONDS 30
#define DS18B20_RESOLUTION 9 

Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht20;
OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature ds18b20(&oneWire);

TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskTelemetryHandle;
TaskHandle_t TaskAudioHandle;
QueueHandle_t healthDataQueue;
SemaphoreHandle_t audioMutex;
WiFiUDP udp;

uint32_t seqNum = 0;
bool ds18b20_ok = false;
bool aht20_ok = false;
bool mic_ok = false;
char device_id[18];

float latestAudioMagnitude = 0.0f;
float latestAudioFeatures[NUM_MEL_BINS] = {0};

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
  uint32_t seq;
  uint32_t timestamp;
  float ax, ay, az;
  float gx, gy, gz;
  float gy_variance;
  float activityJerk;
  bool is_interrupt;
  float bodyTemp; 
  float envTemp;  
  float envHum;   
  BehaviorState state;
  float audioMagnitude;
  float audioFeatures[NUM_MEL_BINS];
};

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
    int le = 1 << l;
    int le2 = le >> 1;
    for (j = 0; j < le2; j++) {
      int idx = j << (m - l);
      int16_t wr = cos_lut[idx];
      int16_t wi = -sin_lut[idx];
      for (int i = j; i < n; i += le) {
        int ip = i + le2;
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
  uint32_t bit = 1UL << 30;
  while (bit > n) bit >>= 2;
  while (bit != 0) {
    if (n >= root + bit) {
      n -= root + bit;
      root = (root >> 1) + bit;
    } else {
      root >>= 1;
    }
    bit >>= 2;
  }
  return root;
}

void IRAM_ATTR mpuInterruptHandler() {
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

void audioTask(void *pvParameters) {
  int16_t raw_buffer[FFT_SIZE];
  int16_t fr[FFT_SIZE];
  int16_t fi[FFT_SIZE];
  esp_task_wdt_add(NULL);
  
  for (;;) {
    esp_task_wdt_reset();
    
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) == 0) {
      continue;
    }
    
    if (!mic_ok) continue;

    i2s_start(I2S_PORT);

    size_t bytes_read = 0;
    i2s_read(I2S_PORT, raw_buffer, sizeof(raw_buffer), &bytes_read, pdMS_TO_TICKS(100));

    uint64_t total_rms_sum = 0;
    uint32_t mel_accumulators[NUM_MEL_BINS] = {0};
    int frames_processed = 0;
    bool read_error = false;

    for (int i = 0; i < 187; i++) {
      if (i % 10 == 0) esp_task_wdt_reset();
      
      esp_err_t res = i2s_read(I2S_PORT, raw_buffer, sizeof(raw_buffer), &bytes_read, pdMS_TO_TICKS(200));
      if (res != ESP_OK) {
        read_error = true;
        mic_ok = false;
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
        uint32_t mag_sq = ((uint32_t)(fr[k] * fr[k])) + ((uint32_t)(fi[k] * fi[k]));
        uint32_t mag = int_sqrt(mag_sq);
        int mel_idx = k / bin_width;
        if (mel_idx >= NUM_MEL_BINS) mel_idx = NUM_MEL_BINS - 1;
        mel_accumulators[mel_idx] += mag;
      }
      frames_processed++;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    i2s_stop(I2S_PORT);

    if (!read_error && frames_processed > 0) {
      uint32_t final_rms = int_sqrt((uint32_t)(total_rms_sum / ((uint64_t)frames_processed * FFT_SIZE)));
      
      xSemaphoreTake(audioMutex, portMAX_DELAY);
      latestAudioMagnitude = (float)final_rms;
      for (int i = 0; i < NUM_MEL_BINS; i++) {
        latestAudioFeatures[i] = (float)mel_accumulators[i] / frames_processed;
      }
      xSemaphoreGive(audioMutex);
    }
  }
}

void sensorTask(void *pvParameters) {
  for (;;) {
    bool urgentMotion = false;
    
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SENSOR_POLL_RATE_MS)) > 0) {
      urgentMotion = true;
      Serial.println("WAKEUP: Interrupt Fired");
    } else {
      Serial.println("WAKEUP: 10s Timer Poll");
    }

    digitalWrite(PIN_LED, LOW);
    
    float bTemp = 0.0f, eTemp = 0.0f, eHum = 0.0f;

    if (ds18b20_ok) {
        bTemp = ds18b20.getTempCByIndex(0);
        ds18b20.requestTemperatures();
        if (bTemp == DEVICE_DISCONNECTED_C || bTemp < -50.0f) {
            bTemp = 0.0f;
            ds18b20_ok = false;
        }
    } else {
        if (ds18b20.getDeviceCount() > 0) {
            ds18b20.setResolution(DS18B20_RESOLUTION);
            ds18b20.requestTemperatures();
            ds18b20_ok = true;
        }
    }

    if (aht20_ok) {
        sensors_event_t humidity, temp;
        if (aht20.getEvent(&humidity, &temp)) {
            eTemp = temp.temperature;
            eHum = humidity.relative_humidity;
        } else {
            aht20_ok = false;
        }
    } else {
        if (aht20.begin()) {
            aht20_ok = true;
        }
    }

    mpu.enableCycle(false);
    mpu.enableSleep(false);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    
    float sumAx = 0, sumAy = 0, sumAz = 0;
    float sumGx = 0, sumGy = 0, sumGz = 0;
    float gyroYSamples[IMU_SAMPLES_PER_CYCLE];
    float totalJerk = 0.0f;
    float prevMag = 0.0f;
    int validSamples = 0;

    for (int i = 0; i < IMU_SAMPLES_PER_CYCLE; i++) {
        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp)) {
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
            prevMag = mag;
            validSamples++;
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_INTERVAL_MS));
    }

    HealthRecord rec = {0};
    rec.timestamp = millis();
    rec.is_interrupt = urgentMotion;
    rec.bodyTemp = bTemp;
    rec.envTemp = eTemp;
    rec.envHum = eHum;

    if (validSamples > 0) {
        rec.ax = sumAx / validSamples;
        rec.ay = sumAy / validSamples;
        rec.az = sumAz / validSamples;
        rec.gx = sumGx / validSamples;
        rec.gy = sumGy / validSamples;
        rec.gz = sumGz / validSamples;

        float mean = rec.gy;
        float variance = 0.0f;
        for (int i = 0; i < validSamples; i++) {
          float diff = gyroYSamples[i] - mean;
          variance += diff * diff;
        }
        rec.gy_variance = variance / validSamples;
    }

    rec.activityJerk = (validSamples > 1) ? totalJerk / (validSamples - 1) : 0.0f;

    if (urgentMotion) {
      rec.state = ALERT_HIGH_ACTIVITY;
      if (mic_ok) {
        xTaskNotifyGive(TaskAudioHandle);
      }
    } else if (rec.activityJerk > JERK_WALKING * 2.0f) {
      rec.state = ALERT_HIGH_ACTIVITY;
    } else if (rec.activityJerk > JERK_WALKING) {
      rec.state = WALKING;
    } else if (rec.activityJerk > JERK_GRAZING) {
      rec.state = GRAZING;
    } else {
      rec.state = RESTING;
    }

    indicateBehavior(rec.state);

    xSemaphoreTake(audioMutex, portMAX_DELAY);
    rec.audioMagnitude = latestAudioMagnitude;
    for (int i = 0; i < NUM_MEL_BINS; i++) {
      rec.audioFeatures[i] = latestAudioFeatures[i];
    }
    latestAudioMagnitude = 0.0f; 
    memset(latestAudioFeatures, 0, sizeof(latestAudioFeatures));
    xSemaphoreGive(audioMutex);

    xQueueSend(healthDataQueue, &rec, pdMS_TO_TICKS(100));

    mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
    mpu.enableCycle(true);

    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(PIN_LED, HIGH);

    if (urgentMotion) {
        mpu.getMotionInterruptStatus();
    }
  }
}

void telemetryTask(void *pvParameters) {
  esp_task_wdt_add(NULL); 
  
  for (;;) {
    esp_task_wdt_reset(); 
    HealthRecord rec;
    
    if (xQueueReceive(healthDataQueue, &rec, pdMS_TO_TICKS(5000)) == pdPASS) {
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi: Connecting...");
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            int retries = 0;
            while (WiFi.status() != WL_CONNECTED && retries < 10) {
                esp_task_wdt_reset(); 
                vTaskDelay(pdMS_TO_TICKS(500));
                retries++;
            }
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("WiFi: Connected!");
            } else {
                Serial.println("WiFi: Failed to connect.");
            }
        }

        if (WiFi.status() == WL_CONNECTED) {
            char af_buf[128] = "[";
            for (int i = 0; i < NUM_MEL_BINS; i++) {
                char temp[16];
                snprintf(temp, sizeof(temp), "%.2f%s", rec.audioFeatures[i], (i == NUM_MEL_BINS - 1) ? "" : ",");
                strlcat(af_buf, temp, sizeof(af_buf));
            }
            strlcat(af_buf, "]", sizeof(af_buf));

            char payload[768];
            snprintf(payload, sizeof(payload), 
                     "{\"id\":\"%s\",\"seq\":%u,\"ts\":%u,\"bt\":%.1f,\"et\":%.1f,\"eh\":%.1f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"gy_var\":%.3f,\"jerk\":%.1f,\"mot_int\":%d,\"err\":0,\"v\":0.00,\"s\":%d,\"af\":%s}", 
                     device_id, seqNum++, rec.timestamp, rec.bodyTemp, rec.envTemp, rec.envHum, 
                     rec.ax, rec.ay, rec.az, rec.gx, rec.gy, rec.gz, rec.gy_variance, 
                     rec.activityJerk, rec.is_interrupt ? 1 : 0, (int)WiFi.RSSI(), af_buf);

            Serial.println("UDP: Sending payload...");
            Serial.println(payload);

            udp.beginPacket(TARGET_IP, UDP_PORT);
            udp.print(payload);
            udp.endPacket();

            Serial.println("UDP: Payload sent.");
        }

        if (uxQueueMessagesWaiting(healthDataQueue) == 0) {
            Serial.println("WiFi: Sleeping.");
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
        }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  esp_pm_config_esp32c3_t pm_config = {
    .max_freq_mhz = 160,
    .min_freq_mhz = 10,
    .light_sleep_enable = true
  };
  esp_pm_configure(&pm_config);

  esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
  
  WiFi.mode(WIFI_STA);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(device_id, sizeof(device_id), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  initDSP(); 

  ds18b20.begin();
  if (ds18b20.getDeviceCount() > 0) {
    ds18b20.setResolution(DS18B20_RESOLUTION);
    ds18b20.requestTemperatures();
    ds18b20_ok = true;
  } 

  if (aht20.begin()) {
    aht20_ok = true;
  }

  if (!mpu.begin()) {
      while(1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setMotionDetectionThreshold(10);
  mpu.setMotionDetectionDuration(2);
  mpu.setInterruptPinLatch(false);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  mpu.enableSleep(false);
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
  mpu.enableCycle(true);

  pinMode(PIN_MPU_INT, INPUT_PULLDOWN);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4, .dma_buf_len = 512, .use_apll = false, .tx_desc_auto_clear = false, .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = { .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS, .data_out_num = I2S_PIN_NO_CHANGE, .data_in_num = I2S_SD };
  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) == ESP_OK) {
    i2s_set_pin(I2S_PORT, &pin_config); 
    i2s_stop(I2S_PORT);
    mic_ok = true; 
  }

  healthDataQueue = xQueueCreate(20, sizeof(HealthRecord));
  audioMutex = xSemaphoreCreateMutex(); 

  xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 3, &TaskSensorHandle);
  xTaskCreate(telemetryTask, "TelemetryTask", 8192, NULL, 2, &TaskTelemetryHandle);
  xTaskCreate(audioTask, "AudioTask", 8192, NULL, 1, &TaskAudioHandle); 

  mpu.getMotionInterruptStatus();

  gpio_wakeup_enable((gpio_num_t)PIN_MPU_INT, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), mpuInterruptHandler, RISING);
}

void loop() {
  vTaskDelete(NULL);
}
