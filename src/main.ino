
#include <math.h>
#include <stdint.h>
#include <stddef.h>

#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include <FastIMU.h>

#include <TinyGPS++.h>

#include "LogEntry.h"


// ------------------- IMU -------------------
#define IMU_ADDRESS 0x68
// #define PERFORM_CALIBRATION
MPU6500 IMU;

calData calib = {0};
AccelData accelData;
GyroData gyroData;


// ------------------- SD Card -------------------
constexpr uint32_t CLOSE_INTERVAL = (10UL * 60UL * 1000UL);  // 10 minutes
constexpr uint32_t FLUSH_INTERVAL = (60UL * 1000UL);         // 1 minute

#define SD_CS_PIN 53
#define SD_CD_PIN -1               // The CD pin is not connected
#define SD_DETECT_ACTIVE_LOW true  // CD goes LOW when a card is inserted

bool sdInserted = false;
File SD_LOG_FILE;


// ------------------- GNSS -------------------
TinyGPSPlus gps;
#define GPS_SERIAL Serial1
#define GPS_BAUD 9600

constexpr uint8_t PPS_PIN = 2;
volatile uint_fast8_t ppsFlag = 0;

void ISR_1PPS() { ppsFlag = 1; }


constexpr size_t ENTRY_SIZE = sizeof(LogEntry);
constexpr size_t BLOCK_SIZE = 512U;
constexpr size_t BLOCK_ENTRIES = BLOCK_SIZE / ENTRY_SIZE;
#define BUFFER_ENTRIES 32

LogEntry ramBuffer[BUFFER_ENTRIES];
uint16_t bufferIdx = 0;
uint32_t seq_counter = 0;


uint32_t lastMillisTick = 0;
uint32_t lastRetryTime = 0;
uint32_t lastCloseTime = 0;
uint32_t lastFlushTime = 0;



static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;

  while (len--) {
    crc ^== (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1
      }
    }
  }

  return crc;
}

static uint32_t gps_datetime_to_unix(int year, int month, int day,
                                     int hour, int minute, int second) {
  int64_t days = days_from_civil(year, month, day);
  int64_t seconds = days * 86400LL + (int64_t)hour * 3600 + minute * 60 + second;
  if (seconds < 0) return 0;
  return (uint32_t)seconds;
}

static inline int16_t clamp_i16(int32_t x) {
  if (x > 32767) return 32767;
  if (x < -32768) return -32768;
  return (int16_t)x;
}

static inline int32_t clamp_i32(int64_t x) {
  if (x > 2147483647LL) return 2147483647;
  if (x < -2147483648LL) return -2147483648;
  return (int32_t)x;
}

static bool ensure_file_open() {
  if (SD_LOG_FILE) return true;
  if (!sdInserted) {
    sdInserted = SD.begin(SD_CS_PIN);
    if (!sdInserted) return false;
  }
  SD_LOG_FILE = SD.open("log.bin", FILE_WRITE);
  return (bool)SD_LOG_FILE;
}

int8_t flush_buffer() {
  if (!sdInserted) return -1;
  if (bufferIdx == 0) return -2;
  if (!ensure_file_open()) return -3;

  size_t bytesToWrite = bufferIdx * ENTRY_SIZE;
  const uint8_t *buf = (const uint8_t *)ramBuffer;
  size_t totalWritten = 0;
  uint32_t start = millis();

  while (totalWritten < bytesToWrite) {
    size_t written = SD_LOG_FILE.write(buf + totalWritten, bytesToWrite - totalWritten);
    if (written == 0) {
      if (millis() - start >= 2000) {
        SD_LOG_FILE.flush();
        return -4;
      }
      continue;
    } 
    totalWritten += written;
  }

  SD_LOG_FILE.flush();
  bufferIdx = 0;
  return 0;
}

bool my_log(const LogEntry &entry) {
  if (bufferIdx >= BUFFER_ENTRIES) {
    int8_t r = flush_buffer();
    if (r != 0) return false;
    if (bufferIdx >= BUFFER_ENTRIES) return false;
  }

  memcpy(&ramBuffer[bufferIdx++], &entry, ENTRY_SIZE);
  return true;
}

static inline bool oneSecondEvent(uint32_t &lastMillisTick) {
  uint32_t now = millis();

  if (ppsFlag) {
    // require a valid GPS fix
    if (gps.time.isValid() && gps.time.age() < 400UL) {
      ppsFlag = 0;
      lastMillisTick = now;
      return true;
    }
    // else keep ppsFlag = 1 and use millis to determine when the one second has passed.
  }
  
  
  if (now - lastMillisTick >= 1000UL) {
    lastMillisTick = now;
    return true;
  }

  return false;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize SD card ??
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  SPI.begin();
  sdInserted = SD.begin(SD_CS_PIN);
  if (sdInserted) {
    SD_LOG_FILE = SD.open("log.bin", FILE_WRITE);
  }

  // Initialize the IMU
  Wire.begin();
  Wire.setClock(400000);

  if (!IMU.init(calib, IMU_ADDRESS)) {
    // TODO: Retry
    // TODO: Log to the SD card that the IMU could not be initialized
  }

#ifdef PERFORM_CALIBRATION
  // Calibrate the IMU
  if (IMU.hasMagnetometer()) {
    delay(1000);
    IMU.calibrateMag(&calib);  // Move the IMU in a figure 8 pattern
    delay(5000);
  }
  IMU.calibrateAccelGyro(&calib);  // Keep the IMU level
  IMU.init(calib, IMU_ADDRESS);
#endif

  // Initialize the GNSS module
  GPS_SERIAL.begin(GPS_BAUD);
  pinMode(PPS_PIN, INPUT); // TODO: OR INPUT_PULLUP ??
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ISR_1PPS, RISING);

  lastMillisTick = millis();
}

void loop() {
  

  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  if (oneSecondEvent(lastMillisTick)) {
    LogEntry e = {0};
    memset(&e, 0, sizeof(e));

    e.seq = ++seq_counter;
    e.t = millis();

    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    // TODO: Ensure |arguments| do not exeed SHORT_MAX!
    e.ax = clamp_i16((int32_t)lroundf(accelData.accelX * 1000.0f));
    e.ay = clamp_i16((int32_t)lroundf(accelData.accelY * 1000.0f));
    e.az = clamp_i16((int32_t)lroundf(accelData.accelZ * 1000.0f));
    e.gx = clamp_i16((int32_t)lroundf(gyroData.gyroX * 10.0f));
    e.gy = clamp_i16((int32_t)lroundf(gyroData.gyroY * 10.0f));
    e.gz = clamp_i16((int32_t)lroundf(gyroData.gyroZ * 10.0f));


    if (gps.location.isValid()) {
      e.speed = lroundf(gps.speed.kmph() * 100.0f);
      e.lat = clamp_i32((int32_t)lroundf(gps.location.lat() * (float)(1e7)));
      e.lng = clamp_i32((int32_t)lroundf(gps.location.lng() * (float)(1e7)));
      e.alt = clamp_i32((int32_t)lroundf(gps.altitude.meters() * 100.0f));
      e.sats = (uint8_t)gps.satellites.value();
    } else {
      e.speed = 0;
      e.lat = e.lng = e.alt = 0;
      e.sats = (uint8_t)gps.satellites.value();
    }


    if (gps.date.isValid() && gps.time.isValid()) {
      e.t_unix = gps_datetime_to_unix((int)gps.date.year()), (int)gps.date.month()),
                                 (int)gps.data.day()), (int)gps.time.hour(),
                                 (int)gps.time.minute(), (int)gps.time.second()));
    } else {
      e.t_unix = 0;
    }

    e.temp = roundf(IMU.getTemp());

    e.crc = 0;
    e.crc = crc16_ccitt((const uint8_t *)&e, offsetof(LogEntry, crc));

    if (!my_log(e)) {
      // TODO: Blink LED.
      // Buffer is full and couldn't flush
    }
  }

  uint32_t now = millis();

  if (now - lastFlushTime >= FLUSH_INTERVAL) {
    lastFlushTime = now;
    (void)flush_buffer();
    // TODO: handle errors
  }

  if (!sdInserted && now - lastRetryTime >= 3000UL) {
    lastRetryTime = now;

    sdInserted = SD.begin(SD_CS_PIN);
    ensure_file_open();
  }

  if (sdInserted && (now - lastCloseTime) >= CLOSE_INTERVAL) {
    lastCloseTime = now;
    flush_buffer();
    if (SD_LOG_FILE) {
      SD_LOG_FILE.close();
      SD_LOG_FILE = File();
    }
  }
}
