#ifndef LOGENTRY_H
#define LOGENTRY_H
#include <stdint.h>

typedef struct __attribute__((packed)) LogEntry {
    uint32_t seq;
    uint32_t t;         // ms
    int32_t speed;      // 100 * km/h
    int16_t ax, ay, az; // 1000 * m/s^2
    int16_t gx, gy, gz; // 10 * deg/s1
    int32_t lat, lng;   // 1e-7 degrees
    int32_t alt;
    uint8_t sats;
    uint32_t t_unix;
    int16_t temp;
    uint8_t reserved[19];
    uint16_t crc;
} LogEntry;

#endif
