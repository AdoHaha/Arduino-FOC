#ifndef SIMPLEFOC_NATIVE_TEST_ARDUINO_H
#define SIMPLEFOC_NATIVE_TEST_ARDUINO_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

typedef uint8_t byte;
typedef uint16_t word;

#define OUTPUT 0x01
#define LOW 0x00
#define HIGH 0x01
#define MSBFIRST 0x01
#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline void delayMicroseconds(unsigned long) {}
inline void delay(unsigned long) {}

#endif
